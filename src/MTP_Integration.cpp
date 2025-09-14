// src/MTP_Integration.cpp
#include "MTP_Integration.h"
extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

namespace
{
    // Globals inside anonymous namespace (internal linkage)
    mtp::Plotter               g_plot;
#if MTP_ENABLE_JITTER
    mtp::JitterTracker*        g_jitter = nullptr;
#endif
    mtp::WatchdogYieldBudget   g_yield(MTP_YIELD_BUDGET_US);
    mtp::OneShotTimer          g_kickTimeout;
    TaskHandle_t               g_pidTaskHandle = nullptr;
    MTP::PidCallback           g_pidCb = nullptr;

    void pid_task_trampoline(void*)
    {
        mtp::RateGovernor rg(MTP_PID_PERIOD_US);
#if MTP_ENABLE_JITTER
        mtp::JitterTracker localJitter(MTP_PID_PERIOD_US);
        g_jitter = &localJitter;
#endif
#if MTP_ENABLE_PLOT
        MTP_PLOT_HEADER(g_plot, MTP_PLOT_HEADER_STR);
#endif
        for (;;)
        {
            rg.wait_next();
#if MTP_ENABLE_JITTER
            localJitter.mark();
#endif
            // user work
            if (g_pidCb)
            {
                g_pidCb();
            }
        }
    }
}

namespace MTP
{

void begin()
{
#if MTP_ENABLE_PLOT
    MTP_PLOT_HEADER(g_plot, MTP_PLOT_HEADER_STR);
#endif
#if MTP_ENABLE_JITTER
    // Defer allocation: g_jitter will be set by pid task once running
#endif
}

bool start_pid_task(PidCallback cb)
{
    g_pidCb = cb;
    if (g_pidTaskHandle) return true;

    BaseType_t ok = xTaskCreatePinnedToCore(
        pid_task_trampoline,
        "PID50ms",
        4096,            // stack
        nullptr,
        tskIDLE_PRIORITY+2,
        &g_pidTaskHandle,
        tskNO_AFFINITY
    );
    return ok == pdPASS;
}

mtp::Plotter& plotter()
{
    return g_plot;
}

mtp::JitterTracker* jitter()
{
#if MTP_ENABLE_JITTER
    return g_jitter;
#else
    return nullptr;
#endif
}

mtp::WatchdogYieldBudget& yield_budget()
{
    return g_yield;
}

void arm_kick_timeout_us(uint64_t us, void(*on_timeout)())
{
    g_kickTimeout.init([on_timeout]()
    {
        if (on_timeout) on_timeout();
    }, "KickTimeout");
    g_kickTimeout.start_after(us);
}

void cancel_kick_timeout()
{
    g_kickTimeout.cancel();
}

template <size_t N>
mtp::DeadlineScheduler<N>& scheduler()
{
    static mtp::DeadlineScheduler<N> sched;
    return sched;
}

// Explicit template instantiations for common sizes
template mtp::DeadlineScheduler<4>&  scheduler<4>();
template mtp::DeadlineScheduler<8>&  scheduler<8>();
template mtp::DeadlineScheduler<16>& scheduler<16>();

} // namespace MTP
