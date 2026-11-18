//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_SYSTEM__REALTIME_HPP_
#define GAITOS_SYSTEM__REALTIME_HPP_

#include <string>

namespace gaitos
{

/**
 * @brief Run the calling thread with SCHED_FIFO
 * @param priority 1 (lowest) to 99 (highest), limited by RLIMIT_RTPRIO
 * @return empty on success, the reason otherwise
 */
std::string set_real_time_priority(int priority);

/**
 * @brief Pin the calling thread to one CPU
 * @return empty on success, the reason otherwise
 */
std::string set_cpu_affinity(int cpu);

/**
 * @brief Lock the current and future memory of the process in RAM (no page faults)
 * @return empty on success, the reason otherwise
 */
std::string lock_memory();

}  // namespace gaitos

#endif  // GAITOS_SYSTEM__REALTIME_HPP_
