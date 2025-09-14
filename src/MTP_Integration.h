// src/MTP_Integration.h
#pragma once
#include <stdint.h>
#include "MTP_Config.h"
#include "MicroTimePlus.h"

namespace MTP
{
// Type for user PID callback called every MTP_PID_PERIOD_US
using PidCallback = void(*)(void);

// Initialize plotting & (optionally) jitter stats
void begin();

// Start a dedicated FreeRTOS task that runs the PID callback at fixed cadence
// Returns true on success.
bool start_pid_task(PidCallback cb);

// Accessors for helpers
mtp::Plotter& plotter();
mtp::JitterTracker* jitter(); // may be null if disabled
mtp::WatchdogYieldBudget& yield_budget();

// One-shot kick timeout helpers
void arm_kick_timeout_us(uint64_t us, void(*on_timeout)());
void cancel_kick_timeout();

// Lightweight scheduler helpers
template <size_t N>
mtp::DeadlineScheduler<N>& scheduler();

} // namespace MTP
