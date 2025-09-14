// MicroTimePlus.h
// Header-only timing & scheduling toolkit for ESP32/ESP-IDF & Arduino-ESP32
// Allman style. Uses esp_timer for precise timers and FreeRTOS-aware sleeps.
// Author: ChatGPT (GPT-5 Thinking) — Public domain / MIT-style; edit as you like.
#pragma once

extern "C"
{
    #include "esp_timer.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <functional>

#ifdef ARDUINO
  #include <Arduino.h>
#endif

namespace mtp
{

// ─────────────────────────────────────────────────────────────
// Time helpers
// ─────────────────────────────────────────────────────────────
static inline int64_t now_us()
{
    return esp_timer_get_time();
}

static inline int64_t now_ms()
{
    return now_us() / 1000;
}

static inline float us_to_ms(float us)
{
    return us / 1000.0f;
}

static inline float us_to_s(float us)
{
    return us / 1e6f;
}

static inline int64_t ms_to_us(int64_t ms)
{
    return ms * 1000;
}

static inline int64_t s_to_us(int64_t s)
{
    return s * 1000000LL;
}

// ─────────────────────────────────────────────────────────────
// RollingStats: mean/variance/min/max via Welford
// Single-producer/task usage.
// ─────────────────────────────────────────────────────────────
class RollingStats
{
public:
    void reset()
    {
        _count = 0;
        _mean = 0.0;
        _m2   = 0.0;
        _minSet = false;
        _min = 0.0;
        _max = 0.0;
    }

    void add(double x)
    {
        _count += 1;
        double delta = x - _mean;
        _mean += delta / (double)_count;
        double delta2 = x - _mean;
        _m2 += delta * delta2;

        if (!_minSet)
        {
            _min = _max = x;
            _minSet = true;
        }
        else
        {
            if (x < _min) _min = x;
            if (x > _max) _max = x;
        }
    }

    int64_t count() const
    {
        return _count;
    }

    double mean() const
    {
        return _mean;
    }

    double variance() const
    {
        return (_count > 1) ? (_m2 / (double)(_count - 1)) : 0.0;
    }

    double stddev() const
    {
        double v = variance();
        return (v > 0.0) ? sqrt(v) : 0.0;
    }

    double min() const
    {
        return _min;
    }

    double max() const
    {
        return _max;
    }

private:
    int64_t _count {0};
    double  _mean {0.0};
    double  _m2   {0.0};
    bool    _minSet {false};
    double  _min {0.0};
    double  _max {0.0};
};

// ─────────────────────────────────────────────────────────────
// MicroStopwatch: elapsed + tick
// ─────────────────────────────────────────────────────────────
class MicroStopwatch
{
public:
    MicroStopwatch()
    {
        reset();
    }

    void reset()
    {
        _start = _lastTick = now_us();
    }

    int64_t elapsed_us() const
    {
        return now_us() - _start;
    }

    float elapsed_ms() const
    {
        return (float)elapsed_us() / 1000.0f;
    }

    float elapsed_s() const
    {
        return (float)elapsed_us() / 1e6f;
    }

    int64_t tick_us()
    {
        int64_t t = now_us();
        int64_t d = t - _lastTick;
        _lastTick = t;
        return d;
    }

private:
    int64_t _start {0};
    int64_t _lastTick {0};
};

// ─────────────────────────────────────────────────────────────
// Precise sleep helpers
// ─────────────────────────────────────────────────────────────
static inline void sleep_us_busy(uint32_t us)
{
    int64_t start = now_us();
    while ((now_us() - start) < (int64_t)us)
    {
        #ifdef ARDUINO
        yield();
        #endif
    }
}

static inline void sleep_us_precise(uint64_t us)
{
    if (us == 0)
    {
        return;
    }

    const uint64_t coarseThreshold = 2000; // us
    if (us > coarseThreshold)
    {
        TickType_t ticks = (TickType_t)((us / 1000) / portTICK_PERIOD_MS);
        if (ticks > 0)
        {
            vTaskDelay(ticks);
        }

        uint64_t spent_ms = (uint64_t)ticks * portTICK_PERIOD_MS;
        uint64_t leftover_us = (us > (spent_ms * 1000)) ? (us - spent_ms * 1000) : 0;
        if (leftover_us > 0)
        {
            sleep_us_busy((uint32_t)leftover_us);
        }
    }
    else
    {
        sleep_us_busy((uint32_t)us);
    }
}

static inline void sleep_until_us(int64_t target_us)
{
    int64_t now = now_us();
    if (target_us > now)
    {
        sleep_us_precise((uint64_t)(target_us - now));
    }
}

// ─────────────────────────────────────────────────────────────
// RateGovernor: fixed-rate loop with drift-free deadlines
// ─────────────────────────────────────────────────────────────
class RateGovernor
{
public:
    explicit RateGovernor(uint32_t period_us)
    {
        set_period_us(period_us);
        _nextDeadline_us = now_us();
        _lastWake_us = now_us();
        _lastActual_dt_us = 0;
    }

    void set_period_us(uint32_t period_us)
    {
        _period_us = period_us;
        _period_ticks = pdMS_TO_TICKS((uint32_t)(period_us / 1000));
        if (_period_ticks == 0 && period_us >= 1000)
        {
            _period_ticks = 1;
        }
    }

    uint32_t period_us() const
    {
        return _period_us;
    }

    uint32_t wait_next()
    {
        int64_t now = now_us();
        _nextDeadline_us += _period_us;

        if (_period_ticks > 0)
        {
            int64_t remain_us = _nextDeadline_us - now;
            if (remain_us > 0)
            {
                uint32_t remain_ms = (uint32_t)(remain_us / 1000);
                TickType_t deltaTicks = pdMS_TO_TICKS(remain_ms);
                if (deltaTicks > 0)
                {
                    vTaskDelay(deltaTicks);
                }
            }
        }

        sleep_until_us(_nextDeadline_us);

        int64_t now2 = now_us();
        _lastActual_dt_us = (uint32_t)(now2 - _lastWake_us);
        _lastWake_us = now2;
        return _lastActual_dt_us;
    }

    uint32_t last_dt_us() const
    {
        return _lastActual_dt_us;
    }

private:
    uint32_t   _period_us {0};
    TickType_t _period_ticks {0};
    int64_t    _nextDeadline_us {0};
    int64_t    _lastWake_us {0};
    uint32_t   _lastActual_dt_us {0};
};

// ─────────────────────────────────────────────────────────────
// ScopedProfiler: RAII scope timer into RollingStats
// ─────────────────────────────────────────────────────────────
class ScopedProfiler
{
public:
    explicit ScopedProfiler(RollingStats& stats)
    : _stats(stats)
    {
        _start = now_us();
    }

    ~ScopedProfiler()
    {
        int64_t dt = now_us() - _start;
        _stats.add((double)dt);
    }

private:
    RollingStats& _stats;
    int64_t       _start {0};
};

// ─────────────────────────────────────────────────────────────
// PeriodicTimer: esp_timer periodic callbacks
// ─────────────────────────────────────────────────────────────
class PeriodicTimer
{
public:
    using Callback = void(*)(void*);
    using Fn       = std::function<void()>;

    PeriodicTimer()
    {
        _handle = nullptr;
        _interval_us = 0;
    }

    ~PeriodicTimer()
    {
        stop();
        if (_handle)
        {
            esp_timer_delete(_handle);
            _handle = nullptr;
        }
    }

    esp_err_t init(Callback cb, void* user, const char* name = "PeriodicTimer")
    {
        if (_handle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        esp_timer_create_args_t args {};
        args.callback = cb;
        args.arg = user;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = name;

        return esp_timer_create(&args, &_handle);
    }

    esp_err_t init(Fn fn, const char* name = "PeriodicTimerFn")
    {
        _fn = fn;

        esp_timer_create_args_t args {};
        args.callback = &PeriodicTimer::trampoline;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = name;

        return esp_timer_create(&args, &_handle);
    }

    esp_err_t start_periodic(uint64_t interval_us)
    {
        if (!_handle)
        {
            return ESP_ERR_INVALID_STATE;
        }
        _interval_us = interval_us;
        return esp_timer_start_periodic(_handle, interval_us);
    }

    esp_err_t stop()
    {
        if (_handle && esp_timer_is_active(_handle))
        {
            return esp_timer_stop(_handle);
        }
        return ESP_OK;
    }

    bool is_running() const
    {
        return _handle && esp_timer_is_active(_handle);
    }

    uint64_t interval_us() const
    {
        return _interval_us;
    }

private:
    static void trampoline(void* arg)
    {
        PeriodicTimer* self = static_cast<PeriodicTimer*>(arg);
        if (self && self->_fn)
        {
            self->_fn();
        }
    }

    esp_timer_handle_t _handle;
    uint64_t           _interval_us;
    Fn                 _fn;
};

// ─────────────────────────────────────────────────────────────
// OneShotTimer: single-shot esp_timer with cancel
// ─────────────────────────────────────────────────────────────
class OneShotTimer
{
public:
    using Callback = void(*)(void*);
    using Fn       = std::function<void()>;

    OneShotTimer()
    {
        _handle = nullptr;
    }

    ~OneShotTimer()
    {
        cancel();
        if (_handle)
        {
            esp_timer_delete(_handle);
            _handle = nullptr;
        }
    }

    esp_err_t init(Callback cb, void* user, const char* name = "OneShotTimer")
    {
        if (_handle)
        {
            return ESP_ERR_INVALID_STATE;
        }
        _cb = cb;
        _user = user;

        esp_timer_create_args_t args {};
        args.callback = cb;
        args.arg = user;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = name;

        return esp_timer_create(&args, &_handle);
    }

    esp_err_t init(Fn fn, const char* name = "OneShotTimerFn")
    {
        _fn = fn;

        esp_timer_create_args_t args {};
        args.callback = &OneShotTimer::trampoline;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = name;

        return esp_timer_create(&args, &_handle);
    }

    esp_err_t start_after(uint64_t delay_us)
    {
        if (!_handle)
        {
            return ESP_ERR_INVALID_STATE;
        }
        return esp_timer_start_once(_handle, delay_us);
    }

    esp_err_t cancel()
    {
        if (_handle && esp_timer_is_active(_handle))
        {
            return esp_timer_stop(_handle);
        }
        return ESP_OK;
    }

    bool is_running() const
    {
        return _handle && esp_timer_is_active(_handle);
    }

private:
    static void trampoline(void* arg)
    {
        OneShotTimer* self = static_cast<OneShotTimer*>(arg);
        if (self && self->_fn)
        {
            self->_fn();
        }
    }

    esp_timer_handle_t _handle;
    Callback           _cb {nullptr};
    void*              _user {nullptr};
    Fn                 _fn;
};

// ─────────────────────────────────────────────────────────────
// DeadlineScheduler: static-capacity timed job queue
// ─────────────────────────────────────────────────────────────
template <size_t Capacity>
class DeadlineScheduler
{
public:
    using Fn = std::function<void()>;

    DeadlineScheduler()
    {
        _timer = nullptr;
        for (size_t i = 0; i < Capacity; ++i)
        {
            _used[i] = false;
            _when[i] = 0;
        }
    }

    ~DeadlineScheduler()
    {
        stop_timer_();
        if (_timer)
        {
            esp_timer_delete(_timer);
            _timer = nullptr;
        }
    }

    int schedule_at(int64_t when_us, Fn fn)
    {
        int idx = find_free_();
        if (idx < 0)
        {
            return -1;
        }
        _used[idx] = true;
        _when[idx] = when_us;
        _fn[idx]   = fn;
        ensure_timer_created_();
        rearm_timer_();
        return idx;
    }

    int schedule_in(uint64_t delay_us, Fn fn)
    {
        return schedule_at(now_us() + (int64_t)delay_us, fn);
    }

    void cancel(int idx)
    {
        if (idx >= 0 && idx < (int)Capacity)
        {
            _used[idx] = false;
        }
        rearm_timer_();
    }

    void cancel_all()
    {
        for (size_t i = 0; i < Capacity; ++i)
        {
            _used[i] = false;
        }
        rearm_timer_();
    }

    int count() const
    {
        int c = 0;
        for (size_t i = 0; i < Capacity; ++i)
        {
            if (_used[i]) ++c;
        }
        return c;
    }

private:
    static void on_timer_trampoline(void* arg)
    {
        DeadlineScheduler* self = static_cast<DeadlineScheduler*>(arg);
        self->on_timer_();
    }

    void ensure_timer_created_()
    {
        if (_timer)
        {
            return;
        }
        esp_timer_create_args_t args {};
        args.callback = &DeadlineScheduler::on_timer_trampoline;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "DeadlineSched";
        esp_timer_create(&args, &_timer);
    }

    void stop_timer_()
    {
        if (_timer && esp_timer_is_active(_timer))
        {
            esp_timer_stop(_timer);
        }
    }

    void rearm_timer_()
    {
        int next = find_next_index_();
        if (next < 0)
        {
            stop_timer_();
            return;
        }

        int64_t now = now_us();
        int64_t when = _when[next];
        uint64_t delay = (when > now) ? (uint64_t)(when - now) : 0ULL;

        stop_timer_();
        if (delay == 0)
        {
            esp_timer_start_once(_timer, 1);
        }
        else
        {
            esp_timer_start_once(_timer, delay);
        }
    }

    void on_timer_()
    {
        int64_t t = now_us();
        for (size_t i = 0; i < Capacity; ++i)
        {
            if (_used[i] && _when[i] <= t)
            {
                auto fn = _fn[i];
                _used[i] = false;
                _when[i] = 0;
                if (fn) fn();
            }
        }
        rearm_timer_();
    }

    int find_free_() const
    {
        for (size_t i = 0; i < Capacity; ++i)
        {
            if (!_used[i]) return (int)i;
        }
        return -1;
    }

    int find_next_index_() const
    {
        int best = -1;
        int64_t bestWhen = 0;
        for (size_t i = 0; i < Capacity; ++i)
        {
            if (_used[i])
            {
                if (best < 0 || _when[i] < bestWhen)
                {
                    best = (int)i;
                    bestWhen = _when[i];
                }
            }
        }
        return best;
    }

private:
    esp_timer_handle_t _timer;
    bool               _used[Capacity];
    int64_t            _when[Capacity];
    Fn                 _fn[Capacity];
};

// ─────────────────────────────────────────────────────────────
// Plotter: Serial Plotter-friendly CSV lines
// ─────────────────────────────────────────────────────────────
class Plotter
{
public:
    void begin(const char* headerCSV = "t_us,dt_us,jitter_us,fps")
    {
        #ifdef ARDUINO
        Serial.println(headerCSV);
        #endif
        _printedHeader = true;
    }

    template <typename... Args>
    void log(Args... args)
    {
        #ifdef ARDUINO
        if (!_printedHeader)
        {
            begin();
        }
        printCSV(args...);
        Serial.println();
        #endif
    }

private:
    template <typename T>
    void printOne(const T& v)
    {
        #ifdef ARDUINO
        Serial.print(v);
        #endif
    }

    template <typename T, typename... Rest>
    void printCSV(const T& v, Rest... rest)
    {
        printOne(v);
        if constexpr (sizeof...(rest) > 0)
        {
            #ifdef ARDUINO
            Serial.print(',');
            #endif
            printCSV(rest...);
        }
    }

private:
    bool _printedHeader {false};
};

#define MTP_PLOT_HEADER(plotter, header) do { (plotter).begin(header); } while(0)
#define MTP_PLOT(plotter, ...) do { (plotter).log(__VA_ARGS__); } while(0)

// ─────────────────────────────────────────────────────────────
// WatchdogYieldBudget: cooperative yielding during long work
// ─────────────────────────────────────────────────────────────
class WatchdogYieldBudget
{
public:
    explicit WatchdogYieldBudget(uint32_t budget_us = 5000)
    : _budget_us(budget_us)
    {
        _lastYield = now_us();
    }

    void set_budget_us(uint32_t budget_us)
    {
        _budget_us = budget_us;
    }

    void reset()
    {
        _lastYield = now_us();
    }

    void maybe_yield()
    {
        int64_t t = now_us();
        if ((uint64_t)(t - _lastYield) >= _budget_us)
        {
            #ifdef ARDUINO
            yield();
            #else
            vTaskDelay(1);
            #endif
            _lastYield = now_us();
        }
    }

private:
    uint32_t _budget_us;
    int64_t  _lastYield;
};

class JitterTracker
{
public:
    explicit JitterTracker(uint64_t expected_us)
    : _expected_us(expected_us)
    {
        _last_us = now_us();
    }

    void mark()
    {
        int64_t t = now_us();
        int64_t dt = t - _last_us;
        _last_us = t;
        double err = (double)dt - (double)_expected_us;
        _stats.add(err);
    }

    const RollingStats& stats() const
    {
        return _stats;
    }

    void reset(uint64_t expected_us)
    {
        _expected_us = expected_us;
        _stats.reset();
        _last_us = now_us();
    }

private:
    uint64_t     _expected_us {0};
    int64_t      _last_us {0};
    RollingStats _stats {};
};

class FPSCounter
{
public:
    FPSCounter()
    {
        _window_start = now_us();
    }

    void tick()
    {
        _count++;
        int64_t t = now_us();
        int64_t dt = t - _window_start;
        if (dt >= s_to_us(1))
        {
            _fps = (float)_count / us_to_s((float)dt);
            _count = 0;
            _window_start = t;
        }
    }

    float fps() const
    {
        return _fps;
    }

private:
    int64_t _window_start {0};
    uint32_t _count {0};
    float _fps {0.0f};
};

} // namespace mtp
