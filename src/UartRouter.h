#pragma once

#include <string>
#include <functional>
#include <vector>

extern "C"
{
    #include "driver/uart.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "esp_log.h"
}

class UartRouter
{
public:
    struct Config
    {
        uart_port_t port = UART_NUM_0;
        int baud = 115200;

        int tx_gpio  = UART_PIN_NO_CHANGE;
        int rx_gpio  = UART_PIN_NO_CHANGE;
        int rts_gpio = UART_PIN_NO_CHANGE;
        int cts_gpio = UART_PIN_NO_CHANGE;

        int rx_buffer_size = 2048;
        int tx_buffer_size = 1024;
        int event_queue_depth = 20;

        char line_delim = '\n';   // 0 disables line parsing
    };

    using LineHandler      = std::function<void(const std::string& line)>;

    // Software substring pattern watcher:
    // token match occurs at 'pos' within 'buffer' (rolling RX window).
    using PatternHandler   = std::function<void(const std::string& buffer, size_t pos)>;

    // Hardware repeated-char pattern (e.g., '+++')
    using HWPatternHandler = std::function<void(const std::string& frame_before_pattern,
                                                int pattern_len)>;

public:
    UartRouter(const Config& cfg);
    ~UartRouter();

    esp_err_t begin();
    void      startTask(const char* name = "uart_router_task",
                        uint32_t stack = 4096,
                        UBaseType_t prio = 12,
                        BaseType_t core = tskNO_AFFINITY);
    void      stopTask();

    int       write(const uint8_t* data, size_t len);
    int       write(const std::string& s);

    // Line parsing callback (if line_delim != 0)
    void      onLine(LineHandler cb);

    // ---------- Software pattern watchers ----------
    // Watch an arbitrary substring in the rolling RX buffer.
    // If consume_to_match=true, bytes up to and including the match are dropped
    // from the internal buffer after the callback (prevents re-triggering on same bytes).
    void      watchPattern(const std::string& token, PatternHandler cb, bool consume_to_match = false);

    // Remove all software pattern watchers
    void      clearPatterns();

    // Limit the rolling RX window size used for substring search (default 4096)
    void      setMaxSoftwareBuffer(size_t bytes);

    // ---------- Hardware pattern detection ----------
    // Detect N repeated occurrences of a single character at baud ISR level.
    // Example: '+' repeated 3 times (classic "+++").
    esp_err_t enableHWPattern(char repeated_chr,
                              uint8_t repeat_count,
                              uint8_t chr_tout = 9,
                              uint8_t post_idle = 0,
                              uint8_t pre_idle = 0,
                              size_t  queue_len = 20,
                              HWPatternHandler cb = nullptr);

private:
    static void taskTrampoline(void* arg);
    void        taskLoop();
    void        handleData(const uint8_t* data, size_t len);
    void        handleLineBuffer();
    void        trySoftwarePatterns();

private:
    struct WatchedPattern
    {
        std::string token;
        PatternHandler cb;
        bool consume_to_match = false;
    };

    Config        cfg_;
    QueueHandle_t event_queue_ = nullptr;
    TaskHandle_t  task_ = nullptr;

    LineHandler   line_cb_ = nullptr;

    // HW pattern
    bool              hw_pattern_enabled_ = false;
    HWPatternHandler  hw_pattern_cb_ = nullptr;
    int               hw_pattern_len_ = 0;   // repeat_count

    // Line parsing
    std::string   line_accum_;

    // Software pattern watchers
    std::string                 rx_accum_;
    std::vector<WatchedPattern> patterns_;
    size_t                      max_buffer_ = 4096;

    static constexpr const char* TAG = "UartRouter";
};
