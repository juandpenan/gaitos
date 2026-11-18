//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_system/RealTime.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <cerrno>
#include <cstring>

namespace gaitos
{

std::string set_real_time_priority(int priority)
{
  sched_param param{};
  param.sched_priority = priority;
  const int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (err != 0) {
    rlimit lim{};
    getrlimit(RLIMIT_RTPRIO, &lim);
    return std::string("SCHED_FIFO priority ") + std::to_string(priority) + ": " +
           std::strerror(err) + " (RLIMIT_RTPRIO " + std::to_string(lim.rlim_cur) + ")";
  }
  return "";
}

std::string set_cpu_affinity(int cpu)
{
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  const int err = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  return err == 0 ? "" : std::string("CPU affinity ") + std::to_string(cpu) + ": " + std::strerror(err);
}

std::string lock_memory()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    return std::string("mlockall: ") + std::strerror(errno);
  }
  return "";
}

}  // namespace gaitos
