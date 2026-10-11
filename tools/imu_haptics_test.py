#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Measure HD-haptics output on the real controller with its own accelerometer.

Plays a haptics test signal to the controller and logs the DualSense motion
sensor while it plays, so the same test can be run over USB and over
Bluetooth (vds) and the actual vibration compared.

  imu_haptics_test.py run --label usb        # controller plugged in
  imu_haptics_test.py run --label bt-now     # over Bluetooth with vdsd
  imu_haptics_test.py analyze usb bt-now     # compare runs

Runs are stored in ./imu-runs/<label>.npz. Keep the controller lying still on
the same surface for every run, and the app's haptics level at 100%.
"""
import argparse
import json
import math
import os
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
import wave

import numpy as np

RATE = 48000
RUNS_DIR = "imu-runs"

# (name, seconds, frequency Hz or None, level dBFS or None)
SEGMENTS = [("lead-in", 1.0, None, None), ("silence", 3.0, None, None)]
for hz in (40, 60, 80, 120, 160, 200, 250, 320):
    SEGMENTS.append((f"{hz} Hz", 2.0, hz, -12.0))
SEGMENTS += [
    ("160 Hz quiet (-46 dBFS)", 2.0, 160, -46.0),
    ("gap", 1.0, None, None),
    ("bright audio", 4.0, "bright", -6.0),
    ("tail", 1.0, None, None),
]


def generate_wav(path):
    """4-channel WAV: speaker (FL/FR) silent, haptics on RL/RR."""
    rng = np.random.default_rng(1234)
    pieces = []
    for _, seconds, kind, level in SEGMENTS:
        n = int(seconds * RATE)
        t = np.arange(n) / RATE
        if kind is None:
            x = np.zeros(n)
        elif kind == "bright":
            # White noise differentiated three times: energy far above 2 kHz,
            # like cymbals in audio-driven haptics. Ideally causes no vibration.
            x = np.diff(rng.standard_normal(n + 3), n=3)
            x /= np.max(np.abs(x))
        else:
            x = np.sin(2 * np.pi * kind * t)
            ramp = min(n // 2, int(0.01 * RATE))
            fade = np.ones(n)
            fade[:ramp] = np.linspace(0, 1, ramp)
            fade[-ramp:] = np.linspace(1, 0, ramp)
            x *= fade
        if level is not None:
            x *= 32767 * 10 ** (level / 20)
        pieces.append(x)
    haptics = np.concatenate(pieces)
    frames = np.zeros((len(haptics), 4), dtype=np.int16)
    frames[:, 2] = frames[:, 3] = np.clip(np.round(haptics), -32768, 32767)
    with wave.open(path, "wb") as out:
        out.setnchannels(4)
        out.setsampwidth(2)
        out.setframerate(RATE)
        out.writeframes(frames.tobytes())


def find_motion_device():
    """evdev node of the DualSense motion sensors (USB or vds virtual)."""
    with open("/proc/bus/input/devices") as f:
        blocks = f.read().split("\n\n")
    for block in blocks:
        name = next((l for l in block.splitlines() if l.startswith("N:")), "")
        if "Motion Sensors" in name and ("DualSense" in name or "Wireless Controller" in name):
            for line in block.splitlines():
                if line.startswith("H:"):
                    for handler in line.split("=", 1)[1].split():
                        if handler.startswith("event"):
                            return "/dev/input/" + handler, name.split("=", 1)[1].strip('"')
    sys.exit("No DualSense motion sensor device found. Is the controller connected?")


def find_sink():
    """PipeWire sink of the controller (real USB DualSense or vds virtual)."""
    nodes = json.loads(subprocess.run(["pw-dump"], capture_output=True, text=True, check=True).stdout)
    for node in nodes:
        props = node.get("info", {}).get("props", {})
        if props.get("media.class") != "Audio/Sink":
            continue
        text = " ".join(str(props.get(k, "")) for k in ("node.name", "node.description", "device.product.name"))
        if "DualSense" in text or "Wireless_Controller" in text or "Wireless Controller" in text:
            return props["node.name"], props.get("node.description", props["node.name"])
    sys.exit("No DualSense audio sink found in PipeWire.")


EVENT = struct.Struct("llHHi")  # struct input_event on 64-bit Linux
EV_SYN, EV_ABS, EV_MSC = 0, 3, 4
ABS_X, ABS_Y, ABS_Z = 0, 1, 2
MSC_TIMESTAMP = 5


def record(device, stop, samples):
    """Appends (wall time, sensor µs, ax, ay, az) at every SYN_REPORT."""
    accel = [0, 0, 0]
    stamp = None
    with open(device, "rb", buffering=0) as f:
        while not stop.is_set():
            ready, _, _ = select.select([f], [], [], 0.1)
            if not ready:
                continue
            data = f.read(EVENT.size * 64)
            for off in range(0, len(data) - EVENT.size + 1, EVENT.size):
                sec, usec, etype, code, value = EVENT.unpack_from(data, off)
                if etype == EV_ABS and code in (ABS_X, ABS_Y, ABS_Z):
                    accel[code] = value
                elif etype == EV_MSC and code == MSC_TIMESTAMP:
                    stamp = value & 0xFFFFFFFF
                elif etype == EV_SYN and code == 0 and stamp is not None:
                    samples.append((sec + usec / 1e6, stamp, *accel))


def cmd_run(args):
    device, device_name = find_motion_device()
    sink, sink_name = (args.sink, args.sink) if args.sink else find_sink()
    print(f"motion sensor: {device_name} ({device})")
    print(f"audio sink:    {sink_name}")
    wav = os.path.join(tempfile.mkdtemp(), "haptics-test.wav")
    generate_wav(wav)
    samples, stop = [], threading.Event()
    recorder = threading.Thread(target=record, args=(device, stop, samples))
    recorder.start()
    time.sleep(0.5)
    print("playing test signal (~%.0f s); keep the controller still..." % sum(s[1] for s in SEGMENTS))
    start = time.time()
    subprocess.run(["pw-play", f"--target={sink}", "--channel-map=FL,FR,RL,RR", wav], check=True)
    time.sleep(0.5)
    stop.set()
    recorder.join()
    data = np.array(samples, dtype=np.float64)
    os.makedirs(RUNS_DIR, exist_ok=True)
    out = os.path.join(RUNS_DIR, args.label + ".npz")
    np.savez(out, data=data, start=start)
    seconds = sensor_seconds(data[:, 1]) if len(data) > 1 else [0.0]
    print(f"recorded {len(data)} sensor samples "
          f"({len(data) / max(seconds[-1], 1e-9):.0f} Hz) -> {out}")


def sensor_seconds(stamps):
    """Sensor timestamps (µs, 32-bit wrapping) -> seconds from the first."""
    d = np.diff(stamps)
    d[d < 0] += 2 ** 32
    return np.concatenate([[0.0], np.cumsum(d)]) / 1e6


def segment_windows():
    t = 0.0
    for name, seconds, kind, level in SEGMENTS:
        yield name, kind, t + 0.3, t + seconds - 0.2
        t += seconds


def amplitude(t, x, hz):
    """Least-squares amplitude of a known frequency (works with uneven and
    sub-Nyquist sampling since the frequency is known)."""
    m = np.column_stack([np.sin(2 * np.pi * hz * t), np.cos(2 * np.pi * hz * t), np.ones_like(t), t - t.mean()])
    coef, *_ = np.linalg.lstsq(m, x, rcond=None)
    return math.hypot(coef[0], coef[1])


def load(label):
    run = np.load(os.path.join(RUNS_DIR, label + ".npz"))
    data, start = run["data"], float(run["start"])
    wall = data[:, 0] - start          # seconds since playback start
    sensor = sensor_seconds(data[:, 1])  # precise sample spacing
    # Map sensor time onto the playback timeline (offset fitted on wall time).
    sensor += np.median(wall - sensor)
    return sensor, data[:, 2:5]


def analyze_run(label):
    t_all, accel = load(label)
    result = {}
    dt = np.diff(t_all)
    result["rate"] = 1.0 / np.median(dt)
    for name, kind, a, b in segment_windows():
        sel = (t_all >= a) & (t_all < b)
        t, x = t_all[sel], accel[sel]
        if len(t) < 20:
            continue
        if isinstance(kind, (int, float)):
            result[name] = math.sqrt(sum(amplitude(t, x[:, k], kind) ** 2 for k in range(3)))
        elif name in ("silence", "bright audio"):
            centered = x - x.mean(axis=0)
            result[name] = float(np.sqrt(np.mean(np.sum(centered ** 2, axis=1))))
    # Sensor noise floor at each test frequency, from the silent segment.
    name, _, a, b = next(w for w in segment_windows() if w[0] == "silence")
    sel = (t_all >= a) & (t_all < b)
    for _, kind, _, _ in segment_windows():
        if isinstance(kind, (int, float)):
            result[("noise", kind)] = math.sqrt(sum(amplitude(t_all[sel], accel[sel][:, k], kind) ** 2 for k in range(3)))
    return result


def db(x):
    return 20 * math.log10(max(x, 1e-9))


def cmd_analyze(args):
    runs = {label: analyze_run(label) for label in args.labels}
    ref = args.labels[0]
    print("sensor rate: " + ", ".join(f"{l} {r['rate']:.0f} Hz" for l, r in runs.items()))
    print(f"\nVibration amplitude at each test tone (dB relative to {ref}; "
          "noise = sensor floor with no signal)")
    header = f"  {'tone':26}" + "".join(f"{l:>12}" for l in args.labels)
    print(header)
    for name, kind, _, _ in segment_windows():
        if not isinstance(kind, (int, float)):
            continue
        base = runs[ref][name]
        cells = []
        for label in args.labels:
            r = runs[label]
            if r[name] < 3 * r[("noise", kind)]:
                cells.append(f"{'(noise)':>12}")
            else:
                cells.append(f"{db(r[name] / base):+11.1f} ")
        print(f"  {name:26}" + "".join(cells) +
              f"   floor {db(runs[ref][('noise', kind)] / base):+.0f} dB")
    print("\nFalse vibration from bright audio (RMS above the silent baseline; lower is better)")
    for label, r in runs.items():
        print(f"  {label:12} {db(r['bright audio'] / r['silence']):+.1f} dB")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="play the test and record the accelerometer")
    run.add_argument("--label", required=True, help="run name, e.g. usb or bt-now")
    run.add_argument("--sink", help="PipeWire sink node.name (auto-detected)")
    analyze = sub.add_parser("analyze", help="compare recorded runs (first is the reference)")
    analyze.add_argument("labels", nargs="+")
    args = parser.parse_args()
    {"run": cmd_run, "analyze": cmd_analyze}[args.command](args)


if __name__ == "__main__":
    main()
