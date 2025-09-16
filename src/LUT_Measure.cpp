#include "LUT_Measure.h"
#include "encoderEncoderPCNT.h"
#include "ESP32_L298N.h"

extern encoderEncoderPCNT encoder;
extern ESP32_L298N        l298n;

extern "C"
{
    #include "driver/pcnt.h"
}

static bool kick_to_target(const MotorIO& io,
                           int           targetDuty,
                           const MeasureCfg& cfg)
{
    if (targetDuty < 0) targetDuty = 0;
    if (targetDuty > io.maxDuty) targetDuty = io.maxDuty;

    // 1) Kick at full duty and wait for any movement
    io.setDuty(io.maxDuty);
    int64_t t0 = mtp::now_us();
    bool moved = false;
    while ((mtp::now_us() - t0) < (int64_t)cfg.kick_timeout_ms * 1000LL)
    {
        if (io.clearEncoderTarget) io.clearEncoderTarget();
        mtp::sleep_us_precise(10'000); // 10ms quick probe
        int c = 0;
        if (io.getCountFrom) c = io.getCountFrom(true);
        if (c > 0)
        {
            moved = true;
            break;
        }
        vTaskDelay(1);
    }

    if (!moved)
    {
        io.setDuty(0);
        return false;
    }

    // 2) Drop to target and confirm continued movement
    io.setDuty(targetDuty);
    if (io.clearEncoderTarget) io.clearEncoderTarget();
    mtp::sleep_us_precise((uint64_t)cfg.after_kick_check_ms * 1000ULL);
    int confirm = (io.getCountFrom) ? io.getCountFrom(true) : 0;
    return (confirm > 0);
}

static float measure_average_ticks(const MotorIO& io,
                                   const MeasureCfg& cfg,
                                   int average_over)
{
    float sum = 0.0f;
    for (int i = 0; i < average_over; ++i)
    {
        if (io.clearEncoderTarget) io.clearEncoderTarget();
        mtp::sleep_us_precise(cfg.window_us);
        int ticks = (io.getCountFrom) ? io.getCountFrom(true) : 0;
        sum += (float)ticks;
    }
    return sum / (float)average_over;
}

static void compute_avg_array(const float* up, const float* down, float* out, int n)
{
    for (int i = 0; i < n; ++i)
    {
        float u = up[i];
        float d = down[i];
        if (u <= 0.0f && d <= 0.0f)      out[i] = 0.0f;
        else if (u <= 0.0f)              out[i] = d;
        else if (d <= 0.0f)              out[i] = u;
        else                              out[i] = 0.5f * (u + d);
    }
}

static int get_max_duty()
{
    int res = (int)l298n.pwm.getResolution();
    if (res <= 0) res = 1024;
    return res - 1;
}

// Concrete motor adapters A/B (using your l298n + encoder objects)
static void setDutyA_impl(int duty) { l298n.A.setDutycycle(duty); }
static void setDutyB_impl(int duty) { l298n.B.setDutycycle(duty); }

static void clearA_impl() { encoder.clearTarget(encoder.pcntA); }
static void clearB_impl() { encoder.clearTarget(encoder.pcntB); }

static int getFromA_impl(bool clear) { return encoder.getCountFrom(encoder.pcntA, clear); }
static int getFromB_impl(bool clear) { return encoder.getCountFrom(encoder.pcntB, clear); }

MotorIO make_motor_io(MotorID id)
{
    MotorIO io {};
    io.maxDuty = get_max_duty();
    if (id == MotorID::A)
    {
        io.setDuty            = &setDutyA_impl;
        io.clearEncoderTarget = &clearA_impl;
        io.getCountFrom       = &getFromA_impl;
    }
    else
    {
        io.setDuty            = &setDutyB_impl;
        io.clearEncoderTarget = &clearB_impl;
        io.getCountFrom       = &getFromB_impl;
    }
    return io;
}

// NVS helpers
static bool nvs_write_blob(nvs_handle_t h, const char* key, const void* data, size_t bytes)
{
    esp_err_t err = nvs_set_blob(h, key, data, bytes);
    if (err != ESP_OK) return false;
    err = nvs_commit(h);
    return err == ESP_OK;
}

static bool persist_results_to_nvs(MotorID id, const both_motors::single_motor& m, const MeasureCfg& cfg)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        printf("[NVS] init error: %s\n", esp_err_to_name(err));
        return false;
    }

    nvs_handle_t h;
    err = nvs_open("lut", NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        printf("[NVS] open error: %s\n", esp_err_to_name(err));
        return false;
    }

    const char axis = (id == MotorID::A) ? 'A' : 'B';
    char key[24];

    snprintf(key, sizeof(key), "%c_down", axis);
    if (!nvs_write_blob(h, key, m.resultDownArray, sizeof(m.resultDownArray)))
    {
        printf("[NVS] write %s failed\n", key);
    }

    snprintf(key, sizeof(key), "%c_up", axis);
    if (!nvs_write_blob(h, key, m.resultUpArray, sizeof(m.resultUpArray)))
    {
        printf("[NVS] write %s failed\n", key);
    }

    snprintf(key, sizeof(key), "%c_avg", axis);
    if (!nvs_write_blob(h, key, m.resultAvgArray, sizeof(m.resultAvgArray)))
    {
        printf("[NVS] write %s failed\n", key);
    }

    snprintf(key, sizeof(key), "%c_start", axis);
    nvs_set_i32(h, key, m.startDutycycleValue);
    snprintf(key, sizeof(key), "%c_stall", axis);
    nvs_set_i32(h, key, m.haltedDutycycleValue);
    snprintf(key, sizeof(key), "%c_safestart", axis);
    nvs_set_i32(h, key, m.safeStartDutycycleValue);

    nvs_set_u32(h, "window_us", cfg.window_us);
    nvs_set_u32(h, "settle_ms", cfg.settle_ms);
    nvs_set_i32(h, "windows_per_pt", cfg.windows_per_point);

    nvs_commit(h);
    nvs_close(h);
    return true;
}

// Printing helpers (two very long lines to keep copy/paste sane)
void print_array_two_rows(const char* label, const float* v, size_t n)
{
    printf("%s\n", label);
    size_t half = n / 2;
    for (size_t pass = 0; pass < 2; ++pass)
    {
        size_t start = pass == 0 ? 0 : half;
        size_t end   = pass == 0 ? half : n;
        for (size_t i = start; i < end; ++i)
        {
            printf("%s%.3f", (i == start ? "" : ","), v[i]);
        }
        printf("\n");
    }
}

void print_array_two_rows_int(const char* label, const int* v, size_t n)
{
    printf("%s\n", label);
    size_t half = n / 2;
    for (size_t pass = 0; pass < 2; ++pass)
    {
        size_t start = pass == 0 ? 0 : half;
        size_t end   = pass == 0 ? half : n;
        for (size_t i = start; i < end; ++i)
        {
            printf("%s%d", (i == start ? "" : ","), v[i]);
        }
        printf("\n");
    }
}

// Startup discovery (find start / stall / safeStart)
static void discover_start_and_stall(const MotorIO& io,
                                     both_motors::single_motor& out,
                                     const MeasureCfg& cfg)
{
    // 1) Find smallest duty that sustains motion after kick
    int startDuty = 0;
    for (int d = 0; d <= io.maxDuty; ++d)
    {
        bool ok = kick_to_target(io, d, cfg);
        if (ok)
        {
            if (io.clearEncoderTarget) io.clearEncoderTarget();
            mtp::sleep_us_precise(cfg.window_us);
            int ticks = (io.getCountFrom) ? io.getCountFrom(true) : 0;
            if (ticks > 0)
            {
                startDuty = d;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    out.startDutycycleValue = startDuty;

    // 2) Ramp up well above start, then slowly decay until stall
    int high = startDuty + 100;
    if (high > io.maxDuty) high = io.maxDuty;
    io.setDuty(high);
    vTaskDelay(pdMS_TO_TICKS(500)); // let it spin strongly

    int stalling = startDuty; // pessimistic initialize
    for (int d = high; d >= 0; --d)
    {
        io.setDuty(d);
        vTaskDelay(pdMS_TO_TICKS(20));

        if (io.clearEncoderTarget) io.clearEncoderTarget();
        mtp::sleep_us_precise(cfg.window_us);
        int ticks = (io.getCountFrom) ? io.getCountFrom(true) : 0;

        if (ticks <= 0)
        {
            // stalled at this duty; the last running duty was d+1
            stalling = d;
            break;
        }
    }
    out.haltedDutycycleValue = stalling;

    // 3) safeStart = stalling + margin (clip)
    int safeStart = stalling + cfg.safe_margin_duty;
    if (safeStart > io.maxDuty) safeStart = io.maxDuty;
    if (safeStart < 0) safeStart = 0;
    out.safeStartDutycycleValue = safeStart;

    // Stop motor after discovery
    io.setDuty(0);

    // Print concise status
    printf("[Startup] start=%d, stall=%d, safeStart=%d (max=%d)\n",
           out.startDutycycleValue, out.haltedDutycycleValue,
           out.safeStartDutycycleValue, io.maxDuty);
}

// Downward LUT: from maxDuty down, stop after first zero result
static void build_down_lut(const MotorIO& io,
                           both_motors::single_motor& out,
                           const MeasureCfg& cfg)
{
    for (int d = io.maxDuty; d >= 0; --d)
    {
        out.dutycycleArray[d] = d;
        io.setDuty(d);
        vTaskDelay(pdMS_TO_TICKS(cfg.settle_ms));

        float avg = measure_average_ticks(io, cfg, cfg.windows_per_point);
        out.resultDownArray[d] = avg;

        if ((d % 64) == 0)
        {
            printf("[Down] d=%4d avg=%.2f\n", d, avg);
        }

        if (avg <= 0.0f)
        {
            // Stop on first zero (rule: single zero is enough)
            for (int k = d - 1; k >= 0; --k)
            {
                out.resultDownArray[k] = 0.0f;
                out.dutycycleArray[k]  = k;
            }
            break;
        }
    }

    io.setDuty(0);
}

// Upward LUT: from safeStart up to max, kicking as needed
static void build_up_lut(const MotorIO& io,
                         both_motors::single_motor& out,
                         const MeasureCfg& cfg)
{
    int start = out.safeStartDutycycleValue;
    if (start < 0) start = 0;
    if (start > io.maxDuty) start = io.maxDuty;

    // Ensure it starts
    if (!kick_to_target(io, start, cfg))
    {
        printf("[Up] initial kick @%d failed, trying upwards...\n", start);
        bool ok = false;
        for (int d = start + 1; d <= io.maxDuty; ++d)
        {
            if (kick_to_target(io, d, cfg))
            {
                printf("[Up] started at d=%d after retries\n", d);
                start = d;
                ok = true;
                break;
            }
        }
        if (!ok)
        {
            printf("[Up] ERROR: could not start motor in upward LUT phase\n");
            io.setDuty(0);
            return;
        }
    }

    // Now measure from start..max
    for (int d = start; d <= io.maxDuty; ++d)
    {
        io.setDuty(d);
        vTaskDelay(pdMS_TO_TICKS(cfg.settle_ms));
        float avg = measure_average_ticks(io, cfg, cfg.windows_per_point);
        out.resultUpArray[d] = avg;
        out.dutycycleArray[d] = d;

        if ((d % 64) == 0)
        {
            printf("[Up]   d=%4d avg=%.2f\n", d, avg);
        }
    }

    io.setDuty(0);
}

// Top-level API
bool measure_motor_lut(MotorID id, both_motors& out, const MeasureCfg& cfg)
{
    // Pause PID task to avoid interference
    MTP::suspend_pid_task();

    MotorIO io = make_motor_io(id);
    auto& slot = (id == MotorID::A) ? out.A : out.B;

    // 0) Clean arrays
    for (int i = 0; i < 1024; ++i)
    {
        slot.dutycycleArray[i]       = i;
        slot.resultFromStallArray[i] = 0.0f;
        slot.resultDownArray[i]      = 0.0f;
        slot.resultUpArray[i]        = 0.0f;
        slot.resultAvgArray[i]       = 0.0f;
    }

    // 1) Startup behavior discovery
    discover_start_and_stall(io, slot, cfg);

    // 2) Downward LUT
    build_down_lut(io, slot, cfg);

    // 3) Upward LUT
    build_up_lut(io, slot, cfg);

    // 4) Average LUT
    compute_avg_array(slot.resultUpArray, slot.resultDownArray, slot.resultAvgArray, 1024);

    // 5) Persist to NVS
    bool ok = persist_results_to_nvs(id, slot, cfg);

    // 6) Print concise outputs (2 long rows each)
    const char axis = (id == MotorID::A) ? 'A' : 'B';
    printf("\n=== LUT RESULTS [%c] ===\n", axis);
    printf("start=%d, stall=%d, safeStart=%d\n",
           slot.startDutycycleValue, slot.haltedDutycycleValue, slot.safeStartDutycycleValue);

    print_array_two_rows_int("duty:", slot.dutycycleArray, 1024);
    print_array_two_rows("down:", slot.resultDownArray, 1024);
    print_array_two_rows("up:  ", slot.resultUpArray, 1024);
    print_array_two_rows("avg: ", slot.resultAvgArray, 1024);
    printf("========================\n\n");

    // Resume PID task
    MTP::resume_pid_task();
    return ok;
}
