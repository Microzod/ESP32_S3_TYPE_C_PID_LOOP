#pragma once
#include <cstring>

extern "C" {
  #include "nvs_flash.h"
  #include "nvs.h"
  #include "esp_err.h"
}

#include "nvsSettings_definition.h"

class SettingsNVS
{
public:
    // Optional: choose your own NVS namespace/key (defaults shown)
    explicit SettingsNVS(const char* ns_name = "ctrl", const char* key_name = "blob")
    : ns_(ns_name), key_(key_name) {}

    // Initialize NVS flash (call once at boot)
    static esp_err_t InitFlash()
    {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        return err;
    }

    // Save/load/erase the whole controlSettings struct
    esp_err_t Save(const controlSettings& s) const
    {
        nvs_handle_t h;
        esp_err_t err = nvs_open(ns_, NVS_READWRITE, &h);
        if (err != ESP_OK) return err;

        Blob b;
        b.magic   = MAGIC;
        b.version = VERSION;
        b.size    = (uint16_t)sizeof(controlSettings);
        std::memcpy(&b.payload, &s, sizeof(controlSettings));

        err = nvs_set_blob(h, key_, &b, sizeof(b));
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        return err;
    }

    esp_err_t Load(controlSettings& s) const
    {
        nvs_handle_t h;
        esp_err_t err = nvs_open(ns_, NVS_READONLY, &h);
        if (err != ESP_OK) return err;

        size_t need = 0;
        err = nvs_get_blob(h, key_, nullptr, &need);
        if (err != ESP_OK) { nvs_close(h); return err; }
        if (need != sizeof(Blob)) { nvs_close(h); return ESP_ERR_INVALID_SIZE; }

        Blob b{};
        size_t len = sizeof(b);
        err = nvs_get_blob(h, key_, &b, &len);
        nvs_close(h);
        if (err != ESP_OK) return err;

        if (b.magic != MAGIC)      return ESP_ERR_INVALID_RESPONSE;
        if (b.version != VERSION)  return ESP_ERR_INVALID_VERSION;
        if (b.size != sizeof(s))    return ESP_ERR_INVALID_SIZE;

        std::memcpy(&s, &b.payload, sizeof(s));
        return ESP_OK;
    }

    esp_err_t Erase() const
    {
        nvs_handle_t h;
        esp_err_t err = nvs_open(ns_, NVS_READWRITE, &h);
        if (err != ESP_OK) return err;
        err = nvs_erase_key(h, key_);
        if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) err = nvs_commit(h);
        nvs_close(h);
        return err;
    }

private:
    struct Blob
    {
        uint32_t        magic;
        uint16_t        version;
        uint16_t        size;     // sizeof(controlSettings) at save time
        controlSettings payload;  // copied verbatim
    };

    static constexpr uint32_t MAGIC   = 0x4354524C; // 'CTRL'
    static constexpr uint16_t VERSION = 1;

    const char* ns_;
    const char* key_;
};
