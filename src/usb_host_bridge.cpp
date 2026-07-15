#include "usb_host_bridge.h"

#include <array>
#include <memory>
#include <cstdio>
#include <cstring>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_types_ch9.h"
#include "usb/usb_types_cdc.h"

#include "usb/vcp.hpp"
#include "usb/vcp_ftdi.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ch34x.hpp"

namespace {
static void updateDriverInfo(const char *name, uint16_t vid, uint16_t pid);
using namespace esp_usb;

static const char *TAG = "usb-host";
static constexpr size_t RX_QUEUE_LEN = 4096;
static constexpr std::array<size_t, 3> RX_QUEUE_FALLBACKS = {RX_QUEUE_LEN, 2048, 1024};
static constexpr size_t TX_TRANSFER_BYTES = 512;
static constexpr TickType_t RX_QUEUE_WAIT_MS = 1000;
static constexpr TickType_t WRITE_TIMEOUT_MS = 50;
static constexpr TickType_t STATE_LOCK_TIMEOUT_MS = 20;
static constexpr BaseType_t USB_TASK_CORE = 0;
static constexpr UBaseType_t USB_EVENT_TASK_PRIORITY = 1;
static constexpr UBaseType_t USB_MONITOR_TASK_PRIORITY = 1;
static constexpr unsigned CDC_DRIVER_TASK_PRIORITY = 1;

static QueueHandle_t s_rxQueue = nullptr;
static size_t s_rxQueueCapacity = 0;
static std::atomic<uint32_t> s_rxDropped{0};
static SemaphoreHandle_t s_stateMutex = nullptr;
static TaskHandle_t s_eventTask = nullptr;
static TaskHandle_t s_monitorTask = nullptr;

static std::unique_ptr<CdcAcmDevice> s_device;
static bool s_initialized = false;
static bool s_connected = false;
static bool s_connectInProgress = false;
static std::atomic_bool s_disconnectFlag{false};

static bool s_supportsLineCoding = true;
static bool s_supportsSignals = true;

static std::atomic<uint32_t> s_lastDeviceId{0};
static uint16_t s_activeVid = 0;
static uint16_t s_activePid = 0;
static char s_driverName[16] = "-";

static cdc_acm_line_coding_t s_lineCoding = {
    .dwDTERate = 115200,
    .bCharFormat = 0,
    .bParityType = 0,
    .bDataBits = 8,
};

struct CallbackCtx {
    QueueHandle_t queue;
};
static const char *charFormatToStr(uint8_t fmt) {
    switch (fmt) {
    case 0: return "1";
    case 1: return "1.5";
    case 2: return "2";
    default: return "?";
    }
}

static const char *parityToStr(uint8_t parity) {
    switch (parity) {
    case 0: return "N";
    case 1: return "O";
    case 2: return "E";
    case 3: return "M";
    case 4: return "S";
    default: return "?";
    }
}

static CallbackCtx s_cbCtx{};

class TaggedFT23x : public FT23x {
public:
    using FT23x::FT23x;
    TaggedFT23x(uint16_t pid, const cdc_acm_host_device_config_t *config, uint8_t interface_idx)
        : FT23x(pid, config, interface_idx)
    {
        updateDriverInfo("FTDI", vid, pid);
    }
};

class TaggedCP210x : public CP210x {
public:
    using CP210x::CP210x;
    TaggedCP210x(uint16_t pid, const cdc_acm_host_device_config_t *config, uint8_t interface_idx)
        : CP210x(pid, config, interface_idx)
    {
        updateDriverInfo("CP210x", vid, pid);
    }
};

class TaggedCH34x : public CH34x {
public:
    using CH34x::CH34x;
    TaggedCH34x(uint16_t pid, const cdc_acm_host_device_config_t *config, uint8_t interface_idx)
        : CH34x(pid, config, interface_idx)
    {
        updateDriverInfo("CH34x", vid, pid);
    }
};

class TaggedGenericCDC : public CdcAcmDevice {
public:
    static constexpr std::array<uint16_t, 1> pids = {0xFFFF};
    static constexpr uint16_t vid = 0xFFFF;

    TaggedGenericCDC(uint16_t, const cdc_acm_host_device_config_t *config, uint8_t interface_idx)
    {
        esp_err_t err = this->open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, interface_idx, config);
        if (err != ESP_OK) {
            throw err;
        }
        uint32_t deviceId = s_lastDeviceId.load(std::memory_order_relaxed);
        uint16_t vidLocal = static_cast<uint16_t>(deviceId >> 16);
        uint16_t pidLocal = static_cast<uint16_t>(deviceId);
        updateDriverInfo("CDC", vidLocal, pidLocal);
        const usb_standard_desc_t *desc = nullptr;
        if (cdc_acm_host_cdc_desc_get(this->cdc_hdl, USB_CDC_DESC_SUBTYPE_ACM, &desc) == ESP_OK && desc) {
            auto *acm = reinterpret_cast<const cdc_acm_acm_desc_t *>(desc);
            if (acm && !acm->bmCapabilities.serial) {
                s_supportsLineCoding = false;
                s_supportsSignals = false;
                ESP_LOGI(TAG, "CDC device has no serial capability bits set");
            }
        }
    }
};

static void updateDriverInfo(const char *name, uint16_t vid, uint16_t pid)
{
    if (!s_stateMutex) {
        return;
    }
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return;
    }
    if (vid || pid) {
        std::snprintf(s_driverName, sizeof(s_driverName), "%s %04X:%04X", name, vid, pid);
    } else {
        std::snprintf(s_driverName, sizeof(s_driverName), "%s", name);
    }
    s_activeVid = vid;
    s_activePid = pid;
    xSemaphoreGive(s_stateMutex);
}

static std::unique_ptr<CdcAcmDevice> detachDeviceLocked()
{
    std::unique_ptr<CdcAcmDevice> detached = std::move(s_device);
    s_connected = false;
    s_activeVid = 0;
    s_activePid = 0;
    std::strncpy(s_driverName, "-", sizeof(s_driverName));
    s_supportsLineCoding = true;
    s_supportsSignals = true;
    return detached;
}

static bool dataCallback(const uint8_t *data, size_t len, void *user_arg)
{
    (void)user_arg;
    if (!s_rxQueue || !data || len == 0) {
        return true;
    }
    TickType_t waitStart = xTaskGetTickCount();
    while (uxQueueSpacesAvailable(s_rxQueue) < len &&
            xTaskGetTickCount() - waitStart < pdMS_TO_TICKS(RX_QUEUE_WAIT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (uxQueueSpacesAvailable(s_rxQueue) < len) {
        s_rxDropped.fetch_add(len, std::memory_order_relaxed);
        return true;
    }

    for (size_t i = 0; i < len; ++i) {
        uint8_t b = data[i];
        if (xQueueSend(s_rxQueue, &b, 0) != pdPASS) {
            s_rxDropped.fetch_add(len - i, std::memory_order_relaxed);
            break;
        }
    }
    return true;
}

static void eventCallback(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (!event) {
        return;
    }
    switch (event->type) {
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        s_disconnectFlag.store(true, std::memory_order_relaxed);
        ESP_LOGI(TAG, "CDC device disconnected");
        break;
    case CDC_ACM_HOST_ERROR:
        ESP_LOGW(TAG, "CDC host error %d", event->data.error);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        break;
    }
}

static void newDeviceCallback(usb_device_handle_t usb_dev)
{
    if (!usb_dev) {
        return;
    }
    const usb_device_desc_t *desc = nullptr;
    if (usb_host_get_device_descriptor(usb_dev, &desc) == ESP_OK && desc) {
        uint32_t deviceId = (static_cast<uint32_t>(desc->idVendor) << 16) | desc->idProduct;
        s_lastDeviceId.store(deviceId, std::memory_order_relaxed);
        ESP_LOGI(TAG, "Detected device VID:PID %04X:%04X", desc->idVendor, desc->idProduct);
    }
}

static cdc_acm_host_device_config_t makeDeviceConfig()
{
    cdc_acm_host_device_config_t cfg = {};
    cfg.connection_timeout_ms = 0; // wait indefinitely
    cfg.out_buffer_size = TX_TRANSFER_BYTES;
    cfg.in_buffer_size = 2048;
    cfg.event_cb = eventCallback;
    cfg.data_cb = dataCallback;
    cfg.user_arg = &s_cbCtx;
    return cfg;
}

static CdcAcmDevice *openDevice()
{
    auto cfg = makeDeviceConfig();
    uint32_t deviceId = s_lastDeviceId.load(std::memory_order_relaxed);
    uint16_t lastVid = static_cast<uint16_t>(deviceId >> 16);
    uint16_t lastPid = static_cast<uint16_t>(deviceId);

    if (lastVid == 0 && lastPid == 0) {
        ESP_LOGI(TAG, "VID/PID not available yet, deferring open");
        return nullptr;
    }

    auto tryDriver = [&](const char *label, auto factory) -> CdcAcmDevice * {
        ESP_LOGI(TAG, "Trying driver %s for %04X:%04X", label, lastVid, lastPid);
        try {
            auto *dev = factory();
            if (dev) {
                ESP_LOGI(TAG, "Driver %s succeeded", label);
            }
            return dev;
        } catch (const std::bad_alloc &) {
            ESP_LOGE(TAG, "Driver %s: OOM", label);
        } catch (const esp_err_t err) {
            ESP_LOGW(TAG, "Driver %s failed: %s", label, esp_err_to_name(err));
        } catch (...) {
            ESP_LOGE(TAG, "Driver %s threw unexpected exception", label);
        }
        return nullptr;
    };

    if (lastVid == TaggedFT23x::vid) {
        if (auto *dev = tryDriver("FTDI", [&]() { return new TaggedFT23x(lastPid, &cfg, 0); })) {
            return dev;
        }
        return nullptr;
    }
    if (lastVid == TaggedCP210x::vid) {
        if (auto *dev = tryDriver("CP210x", [&]() { return new TaggedCP210x(lastPid, &cfg, 0); })) {
            return dev;
        }
        if (auto *dev = tryDriver("CP210x(auto)", [&]() { return new TaggedCP210x(CP210X_PID_AUTO, &cfg, 0); })) {
            return dev;
        }
        return nullptr;
    }
    if (lastVid == TaggedCH34x::vid) {
        if (auto *dev = tryDriver("CH34x", [&]() { return new TaggedCH34x(lastPid, &cfg, 0); })) {
            return dev;
        }
        return nullptr;
    }
    if (lastVid || lastPid) {
        cdc_acm_host_device_config_t targeted = cfg;
        targeted.connection_timeout_ms = 2000;
        if (auto *dev = tryDriver("VCP(vid/pid)", [&]() { return VCP::open(lastVid, lastPid, &targeted); })) {
            return dev;
        }
        if (auto *dev = tryDriver("CDC(generic)", [&]() { return new TaggedGenericCDC(0, &targeted, 0); })) {
            return dev;
        }
        return nullptr;
    }

    try {
        return VCP::open(&cfg);
    } catch (const std::bad_alloc &) {
        ESP_LOGE(TAG, "Out of memory while opening VCP device");
    } catch (const esp_err_t err) {
        if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to open VCP device: %s", esp_err_to_name(err));
        }
    } catch (...) {
        ESP_LOGE(TAG, "Unexpected exception while opening VCP device");
    }
    return nullptr;
}

static void usbEventTask(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t eventFlags = 0;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &eventFlags);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (eventFlags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (eventFlags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed");
        }
        vTaskDelay(1);
    }
}

static void usbMonitorTask(void *arg)
{
    (void)arg;
    // Bewusst NICHT am Task-Watchdog angemeldet:
    //  - Der Task wartet legitim auf USB-Verbindungen und ist niedrig priorisiert.
    //  - Alle Zugriffe des Hauptloops auf den gemeinsamen Zustand sind begrenzt;
    //    ein blockierender Treiber darf daher nur USB, nie Web/Telnet anhalten.
    while (true) {
        if (s_disconnectFlag.exchange(false, std::memory_order_relaxed)) {
            std::unique_ptr<CdcAcmDevice> disconnected;
            if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) == pdTRUE) {
                disconnected = detachDeviceLocked();
                xSemaphoreGive(s_stateMutex);
            } else {
                s_disconnectFlag.store(true, std::memory_order_relaxed);
            }
            if (disconnected) {
                disconnected->close();
                disconnected.reset();
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!s_connected && !s_connectInProgress) {
            s_connectInProgress = true;
            CdcAcmDevice *opened = openDevice();
            if (opened) {
                if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
                    opened->close();
                    delete opened;
                    s_connectInProgress = false;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                s_device.reset(opened);
                s_connected = true;
                xSemaphoreGive(s_stateMutex);
                ESP_LOGI(TAG, "CDC device ready (%s)", s_driverName);
                if (s_supportsLineCoding) {
                    cdc_acm_line_coding_t current{};
                    if (opened->line_coding_get(&current) == ESP_OK) {
                        ESP_LOGI(TAG, "Device default line coding: %u bps, %u%s%s",
                                 (unsigned)current.dwDTERate,
                                 current.bDataBits, parityToStr(current.bParityType), charFormatToStr(current.bCharFormat));
                    } else {
                        ESP_LOGW(TAG, "Failed to read device line coding");
                    }
                }
                usbHostSetLineCoding(0);
                usbHostSetSignals(true, true);
            } else {
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            s_connectInProgress = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace

void usbHostInit()
{
    if (s_initialized) {
        return;
    }
    esp_log_level_set(TAG, ESP_LOG_INFO);

    try {
        VCP::register_driver<TaggedFT23x>();
        VCP::register_driver<TaggedCP210x>();
        VCP::register_driver<TaggedCH34x>();
        VCP::register_driver<TaggedGenericCDC>();
    } catch (const std::bad_alloc &) {
        ESP_LOGE(TAG, "Failed to allocate VCP driver registry");
        return;
    }

    for (size_t capacity : RX_QUEUE_FALLBACKS) {
        s_cbCtx.queue = xQueueCreate(capacity, sizeof(uint8_t));
        if (s_cbCtx.queue) {
            s_rxQueueCapacity = capacity;
            break;
        }
    }
    s_rxQueue = s_cbCtx.queue;
    if (!s_rxQueue) {
        ESP_LOGE(TAG, "Failed to allocate RX queue");
        return;
    }
    if (s_rxQueueCapacity != RX_QUEUE_LEN) {
        ESP_LOGW(TAG, "RX queue fallback: %u bytes", (unsigned)s_rxQueueCapacity);
    }

    s_stateMutex = xSemaphoreCreateMutex();
    if (!s_stateMutex) {
        ESP_LOGE(TAG, "Failed to allocate mutex");
        return;
    }

    usb_host_config_t hostConfig;
    std::memset(&hostConfig, 0, sizeof(hostConfig));
    hostConfig.skip_phy_setup = false;
    hostConfig.root_port_unpowered = false;
    hostConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;
    esp_err_t err = usb_host_install(&hostConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB host install failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_stateMutex);
        vQueueDelete(s_rxQueue);
        s_stateMutex = nullptr;
        s_rxQueue = nullptr;
        s_cbCtx.queue = nullptr;
        s_rxQueueCapacity = 0;
        return;
    }

    const cdc_acm_host_driver_config_t cdcConfig = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = CDC_DRIVER_TASK_PRIORITY,
        .xCoreID = USB_TASK_CORE,
        .new_dev_cb = nullptr,
    };
    err = cdc_acm_host_install(&cdcConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CDC host install failed: %s", esp_err_to_name(err));
        usb_host_uninstall();
        vSemaphoreDelete(s_stateMutex);
        vQueueDelete(s_rxQueue);
        s_stateMutex = nullptr;
        s_rxQueue = nullptr;
        s_cbCtx.queue = nullptr;
        s_rxQueueCapacity = 0;
        return;
    }

    err = cdc_acm_host_register_new_dev_callback(newDeviceCallback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB device callback registration failed: %s", esp_err_to_name(err));
        cdc_acm_host_uninstall();
        usb_host_uninstall();
        vSemaphoreDelete(s_stateMutex);
        vQueueDelete(s_rxQueue);
        s_stateMutex = nullptr;
        s_rxQueue = nullptr;
        s_cbCtx.queue = nullptr;
        s_rxQueueCapacity = 0;
        return;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(
        usbEventTask, "usb_evt", 4096, nullptr,
        USB_EVENT_TASK_PRIORITY, &s_eventTask, USB_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usbEventTask");
        return;
    }
    ok = xTaskCreatePinnedToCore(
        usbMonitorTask, "usb_mon", 4096, nullptr,
        USB_MONITOR_TASK_PRIORITY, &s_monitorTask, USB_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usbMonitorTask");
        return;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "USB host initialised");
}

void usbHostTask() {}

bool usbHostInitialized()
{
    return s_initialized;
}

bool usbHostConnected()
{
    if (!s_stateMutex) {
        return false;
    }
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }
    bool connected = s_connected;
    xSemaphoreGive(s_stateMutex);
    return connected;
}

void usbHostGetDriverName(char *buffer, size_t bufferSize)
{
    if (!buffer || bufferSize == 0) {
        return;
    }
    if (!s_stateMutex) {
        std::snprintf(buffer, bufferSize, "-");
        return;
    }
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        std::snprintf(buffer, bufferSize, "busy");
        return;
    }
    std::snprintf(buffer, bufferSize, "%s", s_driverName);
    xSemaphoreGive(s_stateMutex);
}

int usbHostReadByte()
{
    if (!s_rxQueue) {
        return -1;
    }
    uint8_t value = 0;
    if (xQueueReceive(s_rxQueue, &value, 0) == pdPASS) {
        return static_cast<int>(value);
    }
    return -1;
}

size_t usbHostWrite(const uint8_t *data, size_t len)
{
    if (!s_stateMutex || !data || len == 0) {
        return 0;
    }
    size_t transferLen = std::min(len, TX_TRANSFER_BYTES);
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return 0;
    }
    auto *dev = s_device.get();
    bool ready = s_connected && dev;
    if (ready) {
        err = dev->tx_blocking(data, transferLen, WRITE_TIMEOUT_MS);
    }
    xSemaphoreGive(s_stateMutex);
    if (!ready) {
        return 0;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(err));
        return 0;
    }
    return transferLen;
}

void usbHostWriteByte(uint8_t b)
{
    usbHostWrite(&b, 1);
}

void usbHostSetLineCoding(uint32_t baud_rate)
{
    if (!s_stateMutex) {
        if (baud_rate != 0) {
            s_lineCoding.dwDTERate = baud_rate;
        }
        return;
    }
    esp_err_t err = ESP_ERR_INVALID_STATE;
    bool notSupported = false;
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return;
    }
    if (baud_rate != 0) {
        s_lineCoding.dwDTERate = baud_rate;
    }
    auto *dev = s_device.get();
    bool ready = s_connected && dev;
    if (ready && s_supportsLineCoding) {
        cdc_acm_line_coding_t coding = s_lineCoding;
        err = dev->line_coding_set(&coding);
        if (err == ESP_ERR_NOT_SUPPORTED) {
            s_supportsLineCoding = false;
            notSupported = true;
        }
    }
    xSemaphoreGive(s_stateMutex);
    if (!ready) {
        return;
    }
    if (notSupported) {
        ESP_LOGI(TAG, "Device ignores line coding requests");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set line coding: %s", esp_err_to_name(err));
    }
}

void usbHostSetSignals(bool dtr, bool rts)
{
    if (!s_stateMutex) {
        return;
    }
    esp_err_t err = ESP_ERR_INVALID_STATE;
    bool notSupported = false;
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return;
    }
    auto *dev = s_device.get();
    bool ready = s_connected && dev;
    if (ready && s_supportsSignals) {
        err = dev->set_control_line_state(dtr, rts);
        if (err == ESP_ERR_NOT_SUPPORTED) {
            s_supportsSignals = false;
            notSupported = true;
        }
    }
    xSemaphoreGive(s_stateMutex);
    if (!ready) {
        return;
    }
    if (notSupported) {
        ESP_LOGI(TAG, "Device ignores control-line state");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set control signals: %s", esp_err_to_name(err));
    }
}


uint8_t usbHostDataBits()
{
    if (!s_stateMutex) {
        return s_lineCoding.bDataBits ? s_lineCoding.bDataBits : 8;
    }
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return 8;
    }
    uint8_t bits = s_lineCoding.bDataBits;
    xSemaphoreGive(s_stateMutex);
    return bits ? bits : 8;
}

uint8_t usbHostParity()
{
    if (!s_stateMutex) {
        return s_lineCoding.bParityType;
    }
    if (xSemaphoreTake(s_stateMutex, pdMS_TO_TICKS(STATE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return 0;
    }
    uint8_t parity = s_lineCoding.bParityType;
    xSemaphoreGive(s_stateMutex);
    return parity;
}

uint32_t usbHostDroppedBytes()
{
    return s_rxDropped.load(std::memory_order_relaxed);
}

size_t usbHostRxQueueCapacity()
{
    return s_rxQueueCapacity;
}
