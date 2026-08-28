/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <pthread.h>
#include <sched.h>

#include <iostream>
#include <thread>
#include <vector>

namespace ocs2 {

/**
 * Sets the priority of the input thread.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param thread: A reference to the tread.
 */
inline void setThreadPriority(int priority, pthread_t thread) {
  sched_param sched{};
  sched.sched_priority = priority;

  if (priority != 0) {
    if (pthread_setschedparam(thread, SCHED_FIFO, &sched) != 0) {
      std::cerr << "WARNING: Failed to set threads priority (one possible reason could be "
                   "that the user and the group permissions are not set properly.)"
                << std::endl;
    }
  }
}

/**
 * Sets the priority of the input thread.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param thread: A reference to the tread.
 */
inline void setThreadPriority(int priority, std::thread& thread) {
  setThreadPriority(priority, thread.native_handle());
}

/**
 * Sets the priority of the thread this function is called from.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 */
inline void setThisThreadPriority(int priority) {
  setThreadPriority(priority, pthread_self());
}

/**
 * Pin a thread to the given CPUs. Empty list is a no-op.
 */
inline void setThreadCpuAffinity(pthread_t thread, const std::vector<int>& cpus) {
  if (cpus.empty()) {
    return;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  bool any = false;
  for (const int cpu : cpus) {
    if (cpu >= 0 && cpu < CPU_SETSIZE) {
      CPU_SET(cpu, &set);
      any = true;
    }
  }
  if (!any) {
    return;
  }
  if (pthread_setaffinity_np(thread, sizeof(set), &set) != 0) {
    std::cerr << "WARNING: Failed to set thread CPU affinity\n";
  }
}

inline void setThreadCpuAffinity(std::thread& thread, const std::vector<int>& cpus) {
  setThreadCpuAffinity(thread.native_handle(), cpus);
}

}  // namespace ocs2
