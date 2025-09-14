# MicroTimePlus Integration (ESP32/ESP-IDF + Arduino)

This patch vendors a header-only timing toolkit and thin integration layer:

- Fixed-rate task (50 ms default) using `esp_timer`/FreeRTOS
- Serial Plotter-friendly CSV output (toggle via `MTP_ENABLE_PLOT`)
- Jitter tracking (enable with `MTP_ENABLE_JITTER`)
- One-shot kick timeout helper
- Small deadline scheduler
- Watchdog-friendly yield budget

## Quick start

1. Include headers:
```cpp
#include "MTP_Config.h"
#include "MicroTimePlus.h"
#include "MTP_Integration.h"
```

2. In `setup()`:
```cpp
MTP::begin();
MTP::start_pid_task([]()
{
    // Your 50 ms PID step here: read ticks, compute err, update motor B ...
    // Example CSV:
    // MTP::plotter().log((long long)mtp::now_us(), err, ticksA, ticksB, uB);
});
```

3. Optional: arm a kick timeout when you kick the motor:
```cpp
MTP::arm_kick_timeout_us(150000, []()
{
    // handle timeout: re-kick or escalate state
});
```

## Configuration

- `MTP_PID_PERIOD_US` (default 50000)
- `MTP_ENABLE_PLOT` (1/0)
- `MTP_PLOT_HEADER_STR`
- `MTP_YIELD_BUDGET_US`
- `MTP_ENABLE_JITTER`
