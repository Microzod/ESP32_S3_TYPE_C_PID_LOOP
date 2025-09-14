// include/MTP_Config.h
// Build-time switches for MicroTimePlus integration
// Allman style.
#pragma once

// Set your PID cadence (microseconds). Default 50ms.
#ifndef MTP_PID_PERIOD_US
#define MTP_PID_PERIOD_US 50000UL
#endif

// Enable Serial Plotter logging (1) or disable (0)
#ifndef MTP_ENABLE_PLOT
#define MTP_ENABLE_PLOT 1
#endif

// Plot header (CSV)
#ifndef MTP_PLOT_HEADER_STR
#define MTP_PLOT_HEADER_STR "t_us,err,ticksA,ticksB,uB"
#endif

// Watchdog yield budget during heavy loops (us)
#ifndef MTP_YIELD_BUDGET_US
#define MTP_YIELD_BUDGET_US 4000UL
#endif

// Enable jitter tracking (1/0)
#ifndef MTP_ENABLE_JITTER
#define MTP_ENABLE_JITTER 1
#endif
