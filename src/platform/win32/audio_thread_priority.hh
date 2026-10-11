// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com>

#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <string>
#include <string_view>

#include <windows.h>

namespace vds::win {

/*
 * Raises the calling thread to time-critical priority and the MMCSS "Pro
 * Audio" class for its lifetime. Every thread on the speaker/haptics path
 * needs it, not just the Bluetooth sender: a normal-priority thread that
 * receives or paces USB/IP audio gets delayed by a scheduler quantum or more
 * whenever the machine is busy, and the send queue runs dry.
 */
class AudioThreadPriorityGuard {
public:
  AudioThreadPriorityGuard() {
    previous_priority_ = GetThreadPriority(GetCurrentThread());
    priority_set_ =
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    avrt_ = LoadLibraryA("avrt.dll");
    if (!avrt_) {
      return;
    }

    av_revert_ = reinterpret_cast<AvRevertMmThreadCharacteristicsProc>(
        GetProcAddress(avrt_, "AvRevertMmThreadCharacteristics"));
    const auto av_set = reinterpret_cast<AvSetMmThreadCharacteristicsAProc>(
        GetProcAddress(avrt_, "AvSetMmThreadCharacteristicsA"));
    if (av_set == nullptr || av_revert_ == nullptr) {
      return;
    }

    DWORD task_index = 0;
    mmcss_handle_ = av_set("Pro Audio", &task_index);
    if (mmcss_handle_ != nullptr) {
      mmcss_active_ = true;
    }
  }

  ~AudioThreadPriorityGuard() {
    if (mmcss_active_ && av_revert_ != nullptr) {
      av_revert_(mmcss_handle_);
    }
    if (priority_set_ && previous_priority_ != THREAD_PRIORITY_ERROR_RETURN) {
      SetThreadPriority(GetCurrentThread(), previous_priority_);
    }
    if (avrt_ != nullptr) {
      FreeLibrary(avrt_);
    }
  }

  AudioThreadPriorityGuard(const AudioThreadPriorityGuard &) = delete;
  AudioThreadPriorityGuard &
  operator=(const AudioThreadPriorityGuard &) = delete;

  bool boosted() const { return priority_set_ || mmcss_active_; }

  std::string summary(std::string_view thread) const {
    return std::string(thread) + " thread priority priority_set=" +
           (priority_set_ ? "yes" : "no") +
           " mmcss=" + (mmcss_active_ ? "yes" : "no");
  }

private:
  using AvSetMmThreadCharacteristicsAProc = HANDLE(WINAPI *)(LPCSTR, LPDWORD);
  using AvRevertMmThreadCharacteristicsProc = BOOL(WINAPI *)(HANDLE);

  HMODULE avrt_ = nullptr;
  HANDLE mmcss_handle_ = nullptr;
  AvRevertMmThreadCharacteristicsProc av_revert_ = nullptr;
  int previous_priority_ = THREAD_PRIORITY_ERROR_RETURN;
  bool priority_set_ = false;
  bool mmcss_active_ = false;
};

} // namespace vds::win
