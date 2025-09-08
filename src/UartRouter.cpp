#include "UartRouter.h"
#include <algorithm>

UartRouter::UartRouter(const Config& cfg)
: cfg_(cfg)
{
}

UartRouter::~UartRouter()
{
    stopTask();
    uart_driver_delete(cfg_.port);
}

esp_err_t UartRouter::begin()
{
    uart_config_t uc =
    {
        .baud_rate  = cfg_.baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    ESP_ERROR_CHECK(uart_driver_install(cfg_.port,
                                        cfg_.rx_buffer_size,
                                        cfg_.tx_buffer_size,
                                        cfg_.event_queue_depth,
                                        &event_queue_,
                                        0));

    ESP_ERROR_CHECK(uart_param_config(cfg_.port, &uc));
    ESP_ERROR_CHECK(uart_set_pin(cfg_.port, cfg_.tx_gpio, cfg_.rx_gpio, cfg_.rts_gpio, cfg_.cts_gpio));
    return ESP_OK;
}

void UartRouter::startTask(const char* name, uint32_t stack, UBaseType_t prio, BaseType_t core)
{
    if (task_) return;
    xTaskCreatePinnedToCore(&UartRouter::taskTrampoline, name, stack, this, prio, &task_, core);
}

void UartRouter::stopTask()
{
    if (task_)
    {
        TaskHandle_t t = task_;
        task_ = nullptr;
        vTaskDelete(t);
    }
}

int UartRouter::write(const uint8_t* data, size_t len)
{
    return uart_write_bytes(cfg_.port, (const char*)data, len);
}

int UartRouter::write(const std::string& s)
{
    return write(reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

void UartRouter::onLine(LineHandler cb)
{
    line_cb_ = cb;
}

void UartRouter::watchPattern(const std::string& token, PatternHandler cb, bool consume_to_match)
{
    patterns_.push_back({ token, cb, consume_to_match });
}

void UartRouter::clearPatterns()
{
    patterns_.clear();
}

void UartRouter::setMaxSoftwareBuffer(size_t bytes)
{
    max_buffer_ = bytes ? bytes : 1;
    if (rx_accum_.size() > max_buffer_)
    {
        rx_accum_.erase(0, rx_accum_.size() - max_buffer_);
    }
}

esp_err_t UartRouter::enableHWPattern(char repeated_chr,
                                      uint8_t repeat_count,
                                      uint8_t chr_tout,
                                      uint8_t post_idle,
                                      uint8_t pre_idle,
                                      size_t  queue_len,
                                      HWPatternHandler cb)
{
    hw_pattern_cb_ = cb;
    hw_pattern_len_ = (int)repeat_count;
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(cfg_.port, repeated_chr, repeat_count, chr_tout, post_idle, pre_idle));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(cfg_.port, queue_len));
    hw_pattern_enabled_ = true;
    return ESP_OK;
}

void UartRouter::taskTrampoline(void* arg)
{
    static_cast<UartRouter*>(arg)->taskLoop();
}

void UartRouter::taskLoop()
{
    uart_event_t ev;
    std::vector<uint8_t> tmp(256);

    while (true)
    {
        if (!xQueueReceive(event_queue_, &ev, portMAX_DELAY)) continue;

        switch (ev.type)
        {
            case UART_DATA:
            {
                int to_read = ev.size;
                while (to_read > 0)
                {
                    int chunk = std::min<int>((int)tmp.size(), to_read);
                    int got = uart_read_bytes(cfg_.port, tmp.data(), chunk, 10 / portTICK_PERIOD_MS);
                    if (got > 0)
                    {
                        handleData(tmp.data(), got);
                        to_read -= got;
                    }
                    else
                    {
                        break;
                    }
                }
                break;
            }

            case UART_PATTERN_DET:
            {
                int pos = uart_pattern_pop_pos(cfg_.port);
                if (pos >= 0)
                {
                    std::string frame;
                    frame.resize(pos);
                    int got = uart_read_bytes(cfg_.port, frame.data(), pos, 20 / portTICK_PERIOD_MS);
                    if (got > 0)
                    {
                        if (hw_pattern_cb_)
                        {
                            hw_pattern_cb_(frame, hw_pattern_len_);
                        }
                        else
                        {
                            handleData(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
                        }
                    }

                    // Consume exactly the pattern bytes
                    if (hw_pattern_len_ > 0)
                    {
                        std::vector<uint8_t> dummy(hw_pattern_len_);
                        uart_read_bytes(cfg_.port, dummy.data(), hw_pattern_len_, 10 / portTICK_PERIOD_MS);
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Pattern detected but position not found");
                }
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
            {
                ESP_LOGW(TAG, "UART overflow/buffer full: flushing");
                uart_flush_input(cfg_.port);
                xQueueReset(event_queue_);
                break;
            }

            default:
            {
                // UART_BREAK, UART_PARITY_ERR, UART_FRAME_ERR, etc. could be handled here
                break;
            }
        }
    }
}

void UartRouter::handleData(const uint8_t* data, size_t len)
{
    // Always append to rolling buffer for software patterns
    rx_accum_.append(reinterpret_cast<const char*>(data), len);
    if (rx_accum_.size() > max_buffer_)
    {
        rx_accum_.erase(0, rx_accum_.size() - max_buffer_);
    }

    // Line parsing (optional)
    if (cfg_.line_delim == 0)
    {
        // No line parsing; deliver raw chunk as a line
        if (line_cb_)
        {
            std::string s(reinterpret_cast<const char*>(data), len);
            line_cb_(s);
        }
    }
    else
    {
        for (size_t i = 0; i < len; ++i)
        {
            char c = (char)data[i];
            if (c == cfg_.line_delim)
            {
                handleLineBuffer();
            }
            else
            {
                line_accum_.push_back(c);
                if (line_accum_.size() > max_buffer_)
                {
                    line_accum_.erase(0, line_accum_.size() - max_buffer_);
                }
            }
        }
    }

    // After appending this chunk, try software substring matches
    trySoftwarePatterns();
}

void UartRouter::handleLineBuffer()
{
    std::string line;
    line.swap(line_accum_);
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back(); // trim CR if CRLF
    }
    if (line_cb_)
    {
        line_cb_(line);
    }
}

void UartRouter::trySoftwarePatterns()
{
    if (patterns_.empty() || rx_accum_.empty()) return;

    for (auto& w : patterns_)
    {
        size_t pos = rx_accum_.find(w.token);
        if (pos != std::string::npos)
        {
            if (w.cb)
            {
                w.cb(rx_accum_, pos);
            }

            if (w.consume_to_match)
            {
                size_t cut = pos + w.token.size();
                rx_accum_.erase(0, cut);
            }
            else
            {
                if (pos > 0)
                {
                    rx_accum_.erase(0, pos);
                }
                else
                {
                    rx_accum_.erase(0, 1);
                }
            }
        }
    }

    if (rx_accum_.size() > max_buffer_)
    {
        rx_accum_.erase(0, rx_accum_.size() - max_buffer_);
    }
}
