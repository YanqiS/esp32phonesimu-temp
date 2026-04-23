#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "lwip/inet.h"

// Classic Bluetooth
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_mac.h"

// HFP AG (手机端)
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_hf_ag_api.h"

// PBAP (通讯录同步) - SPP用于RFCOMM通道，内部SDP API用于注册PBAP服务
#include "esp_spp_api.h"
// 需要在 CMakeLists.txt 中添加内部头文件路径（见文件末尾注释）
#include "stack/sdp_api.h"

#define TAG "BT_PHONE"

// ========== Wi-Fi（STA连外网 + AP转发） ==========
#define WIFI_STA_SSID      "YOUR_HOME_WIFI"
#define WIFI_STA_PASS      "YOUR_HOME_PASS"
#define WIFI_AP_SSID       "CarBridge_WIFI"
#define WIFI_AP_PASS       "12345678"
#define WIFI_AP_CHANNEL    6
#define WIFI_MAX_STA_CONN  4
#define WIFI_COUNTRY_CODE  "CN"
// 关闭可显著降低蓝牙内存占用，减少 RFCOMM malloc failed 风险
#define ENABLE_MEDIA_PROFILES 0

// ========== 引脚定义 ==========
// 左旋码（沿用 mcu1_led 配置）
#define BCD1_1 GPIO_NUM_35
#define BCD1_2 GPIO_NUM_34
#define BCD1_4 GPIO_NUM_33
#define BCD1_8 GPIO_NUM_32

// 右旋码（沿用 mcu1_led 配置）
#define BCD2_1 GPIO_NUM_26
#define BCD2_2 GPIO_NUM_25
#define BCD2_4 GPIO_NUM_14
#define BCD2_8 GPIO_NUM_27

// LED指示灯（沿用 mcu1_led 配置）
#define LED_R GPIO_NUM_18  // 红灯 - 通话中
#define LED_G GPIO_NUM_17  // 绿灯 - 已连接
#define LED_B GPIO_NUM_16  // 蓝灯 - 待机

// 按键
#define BOOT_KEY GPIO_NUM_0        // 开关蓝牙
#define CALL_KEY GPIO_NUM_23       // 模拟来电按键

#define DEFAULT_DIAL_NUMBER "13800138000"
#define CNUM_PHONE_NUMBER "18621880000"  // 本机号码，不能和拨出号码重复，否则车机判定为VoIP

// ========== 全局变量 ==========
static char my_mac_id[8];
static char bt_name[32];
static bool bt_on = false;
static esp_netif_t *wifi_sta_netif = NULL;
static esp_netif_t *wifi_ap_netif  = NULL;
static bool sntp_started = false;
static int led_mode = 0;
static bool a2dp_connected = false;
static bool avrcp_connected = false;
static int negotiated_hfp_codec = -1;

// HFP连接状态
static bool hfp_connected = false;
static esp_bd_addr_t connected_device = {0};

// 呼叫状态
typedef enum {
    CALL_STATE_IDLE,        // 空闲
    CALL_STATE_INCOMING,    // 来电中
    CALL_STATE_ACTIVE,      // 通话中
    CALL_STATE_DIALING,     // 拨号中
    CALL_STATE_ALERTING,    // 对方振铃中
} call_state_t;

static call_state_t current_call_state = CALL_STATE_IDLE;
static char current_phone_number[32] = "";
// static TimerHandle_t ring_timer = NULL;
 
// 呼叫方向（用于CLCC响应区分来电/去电）
typedef enum {
    CALL_DIR_OUTGOING,
    CALL_DIR_INCOMING,
} call_direction_t;
static call_direction_t current_call_direction = CALL_DIR_OUTGOING;

// 外拨时 DIALING→ALERTING 的非阻塞定时器
static TimerHandle_t dial_alerting_timer = NULL;
static void dial_alerting_timer_callback(TimerHandle_t xTimer);
 
typedef struct
{
    const char *name;
    const char *number;
} contact_t;

typedef struct
{
    char name[32];
    char number[32];
    char datetime[20]; // UTC格式: YYYYMMDDTHHMMSS
} calllog_t;

static const contact_t phonebook[] = {
    {"张三", "13800138000"},
    {"李四", "13501693774"},
    {"王五", "13600136000"},
    {"赵六", "13700137000"},
};

enum { CALLLOG_MAX = 32 };

static calllog_t incoming_calls[CALLLOG_MAX] = {
};
static size_t incoming_call_count = 0;

static calllog_t outgoing_calls[CALLLOG_MAX] = {
};
static size_t outgoing_call_count = 0;

static calllog_t missed_calls[CALLLOG_MAX] = {
};
static size_t missed_call_count = 0;

static void pbap_get_datetime(char *out, size_t len)
{
    time_t now = time(NULL);
    // 部分板子未配置SNTP/RTC时会返回1970，车机可能直接忽略该时间
    // 回退到一个递增的伪时间，保证字段可被车机识别显示
    static time_t pseudo_now = 1767225600; // 2026-01-01T00:00:00Z
    if (now < 1700000000) { // < 2023-11-14 视为无效系统时钟
        now = pseudo_now++;
    }
    struct tm tm_buf;
    struct tm *ptm = gmtime_r(&now, &tm_buf);
    if (ptm == NULL) {
        strlcpy(out, "20260101T000000", len);
        return;
    }
    strftime(out, len, "%Y%m%dT%H%M%S", ptm);
}

static void pbap_append_calllog(calllog_t *logs, size_t *count,
                                const char *name, const char *number)
{
    if (!logs || !count || !number || number[0] == '\0') return;
    size_t idx = *count;
    if (idx >= CALLLOG_MAX) {
        memmove(&logs[0], &logs[1], sizeof(calllog_t) * (CALLLOG_MAX - 1));
        idx = CALLLOG_MAX - 1;
        *count = CALLLOG_MAX;
    } else {
        *count = idx + 1;
    }

    calllog_t *it = &logs[idx];
    memset(it, 0, sizeof(*it));
    strlcpy(it->name, (name && name[0]) ? name : "未知号码", sizeof(it->name));
    strlcpy(it->number, number, sizeof(it->number));
    pbap_get_datetime(it->datetime, sizeof(it->datetime));
}

static const char *lookup_contact_name(const char *number)
{
    if (number == NULL || number[0] == '\0')
    {
        return NULL;
    }

    for (size_t i = 0; i < sizeof(phonebook) / sizeof(phonebook[0]); ++i)
    {
        if (strcmp(phonebook[i].number, number) == 0)
        {
            return phonebook[i].name;
        }
    }

    return NULL;
}

static void sync_hfp_call_indicators(int call, int callsetup)
{
    if (!hfp_connected)
    {
        return;
    }

    esp_hf_ag_ciev_report(connected_device, ESP_HF_IND_TYPE_CALL, call);
    esp_hf_ag_ciev_report(connected_device, ESP_HF_IND_TYPE_CALLSETUP, callsetup);
    esp_hf_ag_ciev_report(connected_device, ESP_HF_IND_TYPE_SERVICE, 1);
    esp_hf_ag_ciev_report(connected_device, ESP_HF_IND_TYPE_SIGNAL, 5);
}

static void get_hfp_call_snapshot(esp_hf_call_status_t *call, esp_hf_call_setup_status_t *callsetup)
{
    if (call == NULL || callsetup == NULL)
    {
        return;
    }

    switch (current_call_state)
    {
    case CALL_STATE_ACTIVE:
        *call = ESP_HF_CALL_STATUS_CALL_IN_PROGRESS;
        *callsetup = ESP_HF_CALL_SETUP_STATUS_IDLE;
        break;
    case CALL_STATE_INCOMING:
        *call = ESP_HF_CALL_STATUS_NO_CALLS;
        *callsetup = ESP_HF_CALL_SETUP_STATUS_INCOMING;
        break;
    case CALL_STATE_DIALING:
        *call = ESP_HF_CALL_STATUS_NO_CALLS;  // 标准HFP：拨号中call=0
        *callsetup = ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING;
        break;
    case CALL_STATE_ALERTING:
        *call = ESP_HF_CALL_STATUS_NO_CALLS;  // 标准HFP：振铃中call=0
        *callsetup = ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING;
        break;
    case CALL_STATE_IDLE:
    default:
        *call = ESP_HF_CALL_STATUS_NO_CALLS;
        *callsetup = ESP_HF_CALL_SETUP_STATUS_IDLE;
        break;
    }
}

static void respond_current_calls(esp_bd_addr_t remote_addr)
{
    // esp_hf_ag_clcc_response 底层和 CNUM 一样有 ok_flag bug，不发 OK
    // 车机查 CLCC 超时 → 丢弃号码 → 显示"网络电话"
    // 用 unknown_at_send 手工拼 +CLCC 响应 + OK
    // 格式: +CLCC: idx,dir,stat,mode,mpty,"number",type
    char clcc_buf[128];
    int dir;

    if (!hfp_connected)
    {
        ESP_LOGW(TAG, "SLC未建立，跳过CLCC响应");
        return;
    }

    dir = (current_call_direction == CALL_DIR_INCOMING) ? 1 : 0;

    if (current_call_state == CALL_STATE_DIALING)
    {
        ESP_LOGI(TAG, "CLCC返回: 外拨中 %s", current_phone_number);
        snprintf(clcc_buf, sizeof(clcc_buf),
            "+CLCC: 1,%d,2,0,0,\"%s\",129\r\n\r\nOK", dir, current_phone_number);
        esp_hf_ag_unknown_at_send(remote_addr, clcc_buf);
        return;
    }

    if (current_call_state == CALL_STATE_ACTIVE)
    {
        ESP_LOGI(TAG, "CLCC返回: 通话中 %s", current_phone_number);
        snprintf(clcc_buf, sizeof(clcc_buf),
            "+CLCC: 1,%d,0,0,0,\"%s\",129\r\n\r\nOK", dir, current_phone_number);
        esp_hf_ag_unknown_at_send(remote_addr, clcc_buf);
        return;
    }

    if (current_call_state == CALL_STATE_ALERTING)
    {
        ESP_LOGI(TAG, "CLCC返回: 对方振铃 %s", current_phone_number);
        snprintf(clcc_buf, sizeof(clcc_buf),
            "+CLCC: 1,0,3,0,0,\"%s\",129\r\n\r\nOK", current_phone_number);
        esp_hf_ag_unknown_at_send(remote_addr, clcc_buf);
        return;
    }

    if (current_call_state == CALL_STATE_INCOMING)
    {
        ESP_LOGI(TAG, "CLCC返回: 来电中 %s", current_phone_number);
        snprintf(clcc_buf, sizeof(clcc_buf),
            "+CLCC: 1,1,4,0,0,\"%s\",129\r\n\r\nOK", current_phone_number);
        esp_hf_ag_unknown_at_send(remote_addr, clcc_buf);
        return;
    }

    // 空闲时也必须回 OK，否则车机等 OK 超时
    ESP_LOGI(TAG, "当前没有活动呼叫，CLCC返回空");
    esp_hf_ag_unknown_at_send(remote_addr, "\r\nOK");
}

static int read_bcd(gpio_num_t bit1, gpio_num_t bit2, gpio_num_t bit4, gpio_num_t bit8)
{
    int val = 0;
    val |= (gpio_get_level(bit1) == 0) ? 1 : 0;
    val |= (gpio_get_level(bit2) == 0) ? 2 : 0;
    val |= (gpio_get_level(bit4) == 0) ? 4 : 0;
    val |= (gpio_get_level(bit8) == 0) ? 8 : 0;
    return val;
}

// 定时器回调：外拨1秒后从DIALING切到ALERTING（非阻塞）
static void dial_alerting_timer_callback(TimerHandle_t xTimer)
{
    if (current_call_state == CALL_STATE_DIALING && hfp_connected)
    {
        current_call_state = CALL_STATE_ALERTING;
        ESP_LOGI(TAG, "📞 对方振铃中...");
        esp_hf_ag_out_call(
            connected_device, 0, 0,
            ESP_HF_CALL_STATUS_NO_CALLS,
            ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        sync_hfp_call_indicators(0, 3);
    }
}

static esp_err_t bt_init(void);
static void bt_deinit(void);
static void bt_cleanup_partial_init(void);

static esp_err_t configure_bt_identity(void)
{
    esp_err_t ret = esp_bt_gap_set_device_name(bt_name);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置蓝牙名称失败: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_bt_cod_t cod = {
        .major = ESP_BT_COD_MAJOR_DEV_PHONE,
        .minor = 3,  // 3 = Smartphone (0=Uncategorized, 1=Cellular, 2=Cordless)
        .service = ESP_BT_COD_SRVC_TELEPHONY | ESP_BT_COD_SRVC_AUDIO |
                   ESP_BT_COD_SRVC_OBJ_TRANSFER | ESP_BT_COD_SRVC_CAPTURING |
                   ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_NETWORKING |
                   ESP_BT_COD_SRVC_INFORMATION,
    };
    ret = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置设备类别(COD)失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置可发现/可连接失败: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
    ret = esp_bt_gap_set_pin(pin_type, 4, pin_code);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置PIN码失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✓ 设备身份已配置: name=%s, cod=phone/telephony, discoverable=yes", bt_name);
    return ESP_OK;
}

static esp_err_t start_bt_phone(void)
{
    if (bt_on)
    {
        ESP_LOGI(TAG, "蓝牙手机模拟器已启动，无需重复启动");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "👆 启动蓝牙手机模拟器");
    led_mode = 0;

    esp_err_t ret = bt_init();
    if (ret == ESP_OK)
    {
        bt_on = true;
        led_mode = 1; // 蓝灯慢闪（待机）

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "🎉 蓝牙手机模拟器启动成功");
        ESP_LOGI(TAG, "📱 现在应可在手机/车机上搜索到: %s", bt_name);
        ESP_LOGI(TAG, "");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "✗ 蓝牙启动失败: %s", esp_err_to_name(ret));
    led_mode = 5; // 红灯快闪
    return ret;
}

static void start_sntp_if_needed(void)
{
    if (sntp_started) return;
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "ntp.aliyun.com");
    esp_sntp_init();
    sntp_started = true;
    ESP_LOGI(TAG, "🕒 SNTP 已启动，等待时间同步");
}

static void configure_ap_dns_from_sta(void)
{
    if (!wifi_sta_netif || !wifi_ap_netif) return;

    esp_netif_dns_info_t dns_main = {0};
    esp_netif_dns_info_t dns_backup = {0};
    uint32_t offer_dns_addr = 0;
    esp_err_t r1 = esp_netif_get_dns_info(wifi_sta_netif, ESP_NETIF_DNS_MAIN, &dns_main);
    esp_err_t r2 = esp_netif_get_dns_info(wifi_sta_netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

    if (r1 == ESP_OK && dns_main.ip.type == ESP_IPADDR_TYPE_V4 &&
        dns_main.ip.u_addr.ip4.addr != 0) {
        esp_netif_set_dns_info(wifi_ap_netif, ESP_NETIF_DNS_MAIN, &dns_main);
        offer_dns_addr = dns_main.ip.u_addr.ip4.addr;
        ESP_LOGI(TAG, "📶 AP DNS(main) 跟随 STA: " IPSTR, IP2STR(&dns_main.ip.u_addr.ip4));
    } else {
        dns_main.ip.type = ESP_IPADDR_TYPE_V4;
        dns_main.ip.u_addr.ip4.addr = ipaddr_addr("223.5.5.5");
        esp_netif_set_dns_info(wifi_ap_netif, ESP_NETIF_DNS_MAIN, &dns_main);
        offer_dns_addr = dns_main.ip.u_addr.ip4.addr;
        ESP_LOGW(TAG, "⚠️ STA 主DNS不可用，AP DNS(main) 回退到 223.5.5.5");
    }

    if (r2 == ESP_OK && dns_backup.ip.type == ESP_IPADDR_TYPE_V4 &&
        dns_backup.ip.u_addr.ip4.addr != 0) {
        esp_netif_set_dns_info(wifi_ap_netif, ESP_NETIF_DNS_BACKUP, &dns_backup);
        ESP_LOGI(TAG, "📶 AP DNS(backup) 跟随 STA: " IPSTR, IP2STR(&dns_backup.ip.u_addr.ip4));
    } else {
        dns_backup.ip.type = ESP_IPADDR_TYPE_V4;
        dns_backup.ip.u_addr.ip4.addr = ipaddr_addr("114.114.114.114");
        esp_netif_set_dns_info(wifi_ap_netif, ESP_NETIF_DNS_BACKUP, &dns_backup);
        ESP_LOGW(TAG, "⚠️ STA 备DNS不可用，AP DNS(backup) 回退到 114.114.114.114");
    }

    // 通过 DHCP Option 6 显式下发 DNS，避免部分车机只认 DHCP 提供的 DNS
    esp_err_t stop_ret = esp_netif_dhcps_stop(wifi_ap_netif);
    if (stop_ret != ESP_OK && stop_ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(TAG, "⚠️ 停止 AP DHCP Server 失败: %s", esp_err_to_name(stop_ret));
    }
    dhcps_offer_t dhcps_dns_offer = OFFER_DNS;
    esp_err_t opt_ret = esp_netif_dhcps_option(wifi_ap_netif, ESP_NETIF_OP_SET,
                                               ESP_NETIF_DOMAIN_NAME_SERVER,
                                               &dhcps_dns_offer, sizeof(dhcps_dns_offer));
    if (opt_ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ 设置 DHCP DNS Option 失败: %s", esp_err_to_name(opt_ret));
    } else {
        esp_ip4_addr_t offer_dns_ip = {.addr = offer_dns_addr};
        ESP_LOGI(TAG, "📶 DHCP DNS Option 已设置: " IPSTR, IP2STR(&offer_dns_ip));
    }
    esp_err_t start_ret = esp_netif_dhcps_start(wifi_ap_netif);
    if (start_ret != ESP_OK && start_ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGW(TAG, "⚠️ 启动 AP DHCP Server 失败: %s", esp_err_to_name(start_ret));
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Wi-Fi STA 断开，reason=%d，尝试重连", disc ? disc->reason : -1);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi STA 已连上，IP=" IPSTR, IP2STR(&event->ip_info.ip));
        configure_ap_dns_from_sta();
        start_sntp_if_needed();
#if CONFIG_LWIP_IPV4_NAPT
        if (wifi_ap_netif) {
            esp_err_t napt_ret = esp_netif_napt_enable(wifi_ap_netif);
            if (napt_ret == ESP_OK || napt_ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGI(TAG, "📶 AP NAT 已开启（可通过 %s 共享上网）", WIFI_AP_SSID);
            } else {
                ESP_LOGW(TAG, "⚠️ AP NAT 开启失败: %s", esp_err_to_name(napt_ret));
            }
        }
#else
        ESP_LOGW(TAG, "⚠️ 未启用 CONFIG_LWIP_IPV4_NAPT，AP 客户端可能显示“网络不可用”");
#endif
    }
}

static esp_err_t wifi_init_sta_ap(void)
{
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    wifi_sta_netif = esp_netif_create_default_wifi_sta();
    wifi_ap_netif  = esp_netif_create_default_wifi_ap();
    if (!wifi_sta_netif || !wifi_ap_netif) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t sta_cfg = {0};
    strlcpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, WIFI_STA_PASS, sizeof(sta_cfg.sta.password));
    // 放宽到 WPA，兼容 WPA/WPA2/WPA3 混合网络，避免因阈值过高导致拒连
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_cfg.sta.pmf_cfg.capable = true;
    sta_cfg.sta.pmf_cfg.required = false;

    wifi_config_t ap_cfg = {0};
    strlcpy((char *)ap_cfg.ap.ssid, WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid));
    strlcpy((char *)ap_cfg.ap.password, WIFI_AP_PASS, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len = strlen(WIFI_AP_SSID);
    ap_cfg.ap.channel = WIFI_AP_CHANNEL;
    ap_cfg.ap.max_connection = WIFI_MAX_STA_CONN;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (strlen(WIFI_AP_PASS) == 0) ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    esp_err_t ctry_ret = esp_wifi_set_country_code(WIFI_COUNTRY_CODE, true);
    if (ctry_ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ 设置国家码失败: %s", esp_err_to_name(ctry_ret));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    if (strcmp(WIFI_STA_SSID, "YOUR_HOME_WIFI") == 0) {
        ESP_LOGW(TAG, "⚠️ 你还没改 WIFI_STA_SSID/WIFI_STA_PASS，当前一定会连不上");
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    // 关闭省电，优先保证首次连网稳定性和流媒体实时性（音乐App更敏感）
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "📶 Wi-Fi AP+STA 已启动");
    ESP_LOGI(TAG, "📶 STA 连接: ssid=%s", WIFI_STA_SSID);
    ESP_LOGI(TAG, "📶 AP 热点: ssid=%s pass=%s", WIFI_AP_SSID, WIFI_AP_PASS);
    return ESP_OK;
}

static void a2dp_source_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        a2dp_connected = (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED);
        ESP_LOGI(TAG, "A2DP连接状态: %d", param->conn_stat.state);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP音频状态: %d", param->audio_stat.state);
        break;
    default:
        ESP_LOGD(TAG, "A2DP事件: %d", event);
        break;
    }
}

static void avrc_tg_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event)
    {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        avrcp_connected = param->conn_stat.connected;
        ESP_LOGI(TAG, "AVRCP TG连接状态: %d", param->conn_stat.connected);
        break;
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        ESP_LOGI(TAG, "AVRCP音量命令: %d", param->set_abs_vol.volume);
        break;
    default:
        ESP_LOGD(TAG, "AVRCP TG事件: %d", event);
        break;
    }
}

/* ===================== LED控制 ===================== */

static inline void led_off(void)
{
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 1);
}

static inline void led_red(void)
{
    gpio_set_level(LED_R, 0);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 1);
}

static inline void led_green(void)
{
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 0);
    gpio_set_level(LED_B, 1);
}

static inline void led_blue(void)
{
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 0);
}

static void led_task(void *arg)
{
    while (1)
    {
        if (led_mode == 1)
        {
            // 待机 - 蓝灯慢闪
            led_blue();
            vTaskDelay(pdMS_TO_TICKS(500));
            led_off();
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else if (led_mode == 2)
        {
            // 已连接 - 绿灯常亮
            led_green();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (led_mode == 3)
        {
            // 来电 - 绿灯快闪
            led_green();
            vTaskDelay(pdMS_TO_TICKS(200));
            led_off();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        else if (led_mode == 4)
        {
            // 通话中 - 红灯常亮
            led_red();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (led_mode == 5)
        {
            // 错误 - 红灯快闪
            led_red();
            vTaskDelay(pdMS_TO_TICKS(100));
            led_off();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            led_off();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ===================== 呼叫管理 ===================== */

// // 定时发送RING
// static void ring_timer_callback(TimerHandle_t xTimer)
// {
//     if (current_call_state == CALL_STATE_INCOMING && hfp_connected)
//     {
//         ESP_LOGI(TAG, "🔔 发送RING...");
//         // 持续发送呼叫指示
//         esp_hf_ag_ciev_report(connected_device, ESP_HF_IND_TYPE_CALL, 1);
//     }
// }

// 模拟来电
void simulate_incoming_call(const char *phone_number)
{
    if (!hfp_connected)
    {
        ESP_LOGW(TAG, "❌ HFP未连接，无法模拟来电");
        return;
    }

    if (current_call_state != CALL_STATE_IDLE)
    {
        ESP_LOGW(TAG, "❌ 当前有通话，无法模拟新来电");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📞 ========== 模拟来电 ==========");
    ESP_LOGI(TAG, "📞 来电号码: %s", phone_number);
    const char *name = lookup_contact_name(phone_number);
    if (name != NULL)
    {
        ESP_LOGI(TAG, "📞 联系人: %s", name);
    }
    ESP_LOGI(TAG, "📞 ===============================");

    // 保存电话号码
    strncpy(current_phone_number, phone_number, sizeof(current_phone_number) - 1);
    current_call_state = CALL_STATE_INCOMING;
    current_call_direction = CALL_DIR_INCOMING;

    // 更新LED
    led_mode = 3; // 绿灯快闪

    // 关键修复：用 esp_hf_ag_answer_call 把 setup 切到 INCOMING。
    // 虽然函数名叫 answer_call，实际上它是 AG call-state 变更的统一入口。
    // stack 内部会根据 (call=0, setup=INCOMING) 自动发出 +CIEV callsetup=1、
    // RING 以及 +CLIP:"<num>",129，车机才会弹出来电界面。
    // 之前直接调用 ciev_report 不会走 RING/CLIP 的发送流程。
    esp_hf_ag_answer_call(
        connected_device,
        0,                                    // num_active
        0,                                    // num_held
        ESP_HF_CALL_STATUS_NO_CALLS,          // call = 0
        ESP_HF_CALL_SETUP_STATUS_INCOMING,    // callsetup = 1 (触发 RING/CLIP)
        current_phone_number,
        ESP_HF_CALL_ADDR_TYPE_UNKNOWN);

    ESP_LOGI(TAG, "💡 已通知车机来电，stack 会自动发送 RING/+CLIP，等待车机接听/拒接...");
}

// 接听来电
void handle_call_answer(void)
{
    if (dial_alerting_timer != NULL) xTimerStop(dial_alerting_timer, 0);

    if (current_call_state != CALL_STATE_INCOMING &&
        current_call_state != CALL_STATE_ALERTING &&
        current_call_state != CALL_STATE_DIALING)
    {
        ESP_LOGW(TAG, "❌ 当前没有可接通的呼叫");
        return;
    }

    if (current_call_state == CALL_STATE_ALERTING || current_call_state == CALL_STATE_DIALING)
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "✅ ========== 对方接听 ==========");
        ESP_LOGI(TAG, "✅ 电话号码: %s", current_phone_number);
        ESP_LOGI(TAG, "✅ ===============================");

        current_call_state = CALL_STATE_ACTIVE;
        led_mode = 4;

        esp_hf_ag_out_call(
            connected_device,
            1,
            0,
            ESP_HF_CALL_STATUS_CALL_IN_PROGRESS,
            ESP_HF_CALL_SETUP_STATUS_IDLE,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);

        // sync_hfp_call_indicators(1, 0);
        esp_hf_ag_audio_connect(connected_device);
        ESP_LOGI(TAG, "🎙️ 已切换到通话中，等待音频链路建立...");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✅ ========== 接听来电 ==========");
    ESP_LOGI(TAG, "✅ 电话号码: %s", current_phone_number);
    const char *name = lookup_contact_name(current_phone_number);
    if (name != NULL)
    {
        ESP_LOGI(TAG, "✅ 联系人: %s", name);
    }
    ESP_LOGI(TAG, "✅ ===============================");

    // // 停止RING
    // if (ring_timer != NULL)
    // {
    //     xTimerStop(ring_timer, 0);
    // }

    current_call_state = CALL_STATE_ACTIVE;
    led_mode = 4; // 红灯常亮

    // 发送接听应答
    esp_hf_ag_answer_call(
        connected_device,
        1,                                    // num_active=1 (1个活动呼叫)
        0,                                    // num_held=0
        ESP_HF_CALL_STATUS_CALL_IN_PROGRESS,
        ESP_HF_CALL_SETUP_STATUS_IDLE,
        current_phone_number,
        ESP_HF_CALL_ADDR_TYPE_UNKNOWN
    );

    // // 发送呼叫状态更新
    // sync_hfp_call_indicators(1, 0);

    // 建立SCO音频连接
    esp_hf_ag_audio_connect(connected_device);

    ESP_LOGI(TAG, "🎙️ 通话已建立，音频连接中...");
}

// 拒接来电
void handle_call_reject(void)
{
    if (dial_alerting_timer != NULL) xTimerStop(dial_alerting_timer, 0);

    if (current_call_state != CALL_STATE_INCOMING)
    {
        ESP_LOGW(TAG, "❌ 当前无来电，无法拒接");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "❌ ========== 拒接来电 ==========");
    ESP_LOGI(TAG, "❌ 电话号码: %s", current_phone_number);
    ESP_LOGI(TAG, "❌ ===============================");

    const char *name = lookup_contact_name(current_phone_number);
    pbap_append_calllog(missed_calls, &missed_call_count, name, current_phone_number);

    // // 停止RING
    // if (ring_timer != NULL)
    // {
    //     xTimerStop(ring_timer, 0);
    // }

    current_call_state = CALL_STATE_IDLE;
    led_mode = 2; // 绿灯常亮

    // 发送拒接应答
    esp_hf_ag_reject_call(
        connected_device,
        0,                                    // num_active=0
        0,                                    // num_held=0
        ESP_HF_CALL_STATUS_NO_CALLS,
        ESP_HF_CALL_SETUP_STATUS_IDLE,
        current_phone_number,
        ESP_HF_CALL_ADDR_TYPE_UNKNOWN
    );

    sync_hfp_call_indicators(0, 0);

    memset(current_phone_number, 0, sizeof(current_phone_number));
    ESP_LOGI(TAG, "📵 来电已拒绝");
}

// 挂断电话
void handle_call_hangup(void)
{
    if (dial_alerting_timer != NULL) xTimerStop(dial_alerting_timer, 0);

    if (current_call_state != CALL_STATE_ACTIVE)
    {
        ESP_LOGW(TAG, "❌ 当前无通话，无法挂断");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📴 ========== 结束通话 ==========");
    ESP_LOGI(TAG, "📴 电话号码: %s", current_phone_number);
    const char *name = lookup_contact_name(current_phone_number);
    if (name != NULL)
    {
        ESP_LOGI(TAG, "📴 联系人: %s", name);
    }
    ESP_LOGI(TAG, "📴 ===============================");

    // 通话结束后写入通话记录：来电通话→ICH，外拨通话→OCH
    if (current_call_direction == CALL_DIR_INCOMING) {
        pbap_append_calllog(incoming_calls, &incoming_call_count, name, current_phone_number);
    } else {
        pbap_append_calllog(outgoing_calls, &outgoing_call_count, name, current_phone_number);
    }

    current_call_state = CALL_STATE_IDLE;
    led_mode = 2; // 绿灯常亮

    // 发送挂断应答
    esp_hf_ag_end_call(
        connected_device,
        0,                                    // num_active=0
        0,                                    // num_held=0
        ESP_HF_CALL_STATUS_NO_CALLS,
        ESP_HF_CALL_SETUP_STATUS_IDLE,
        current_phone_number,
        ESP_HF_CALL_ADDR_TYPE_UNKNOWN
    );

    sync_hfp_call_indicators(0, 0);

    // 断开SCO音频
    esp_hf_ag_audio_disconnect(connected_device);

    memset(current_phone_number, 0, sizeof(current_phone_number));
    ESP_LOGI(TAG, "📵 通话已结束");
}

// 外拨电话
void handle_call_dial(const char *number)
{
    if (!hfp_connected)
    {
        ESP_LOGW(TAG, "❌ HFP未连接，无法拨号");
        return;
    }

    if (current_call_state != CALL_STATE_IDLE)
    {
        ESP_LOGW(TAG, "❌ 当前有通话，无法拨号");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📞 ========== 外拨电话 ==========");
    ESP_LOGI(TAG, "📞 拨号: %s", number);
    const char *name = lookup_contact_name(number);
    if (name != NULL)
    {
        ESP_LOGI(TAG, "📞 联系人: %s", name);
    }
    ESP_LOGI(TAG, "📞 ===============================");

    strncpy(current_phone_number, number, sizeof(current_phone_number) - 1);
    current_call_state = CALL_STATE_DIALING;
    current_call_direction = CALL_DIR_OUTGOING;
    led_mode = 3; // 绿灯快闪

    // 发送外拨应答
    esp_hf_ag_out_call(
        connected_device,
        0,                                    // num_active=0
        0,                                    // num_held=0
        ESP_HF_CALL_STATUS_NO_CALLS,
        ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING,    // 2
        current_phone_number,
        ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    sync_hfp_call_indicators(0, 2);

    // 用定时器延迟1秒切到ALERTING，避免阻塞BT协议栈回调
    if (dial_alerting_timer == NULL)
    {
        dial_alerting_timer = xTimerCreate("dial_alert", pdMS_TO_TICKS(1000),
            pdFALSE, NULL, dial_alerting_timer_callback);
    }
    if (dial_alerting_timer != NULL)
    {
        xTimerStop(dial_alerting_timer, 0);
        xTimerStart(dial_alerting_timer, 0);
    }
    else
    {
        ESP_LOGW(TAG, "⚠️ 定时器不可用，立即切换到对方振铃");
        dial_alerting_timer_callback(NULL);
    }

    ESP_LOGI(TAG, "💡 等待对端接听：板子旋钮2→0可接通，旋钮3→0可取消");
}

/* ===================== HFP AG事件回调 ===================== */

static void hfp_ag_callback(esp_hf_cb_event_t event, esp_hf_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HF_CONNECTION_STATE_EVT:
    {
        uint8_t *bda = param->conn_stat.remote_bda;
        ESP_LOGI(TAG, "HFP连接状态: state=%d, peer_feat=0x%" PRIx32 ", chld_feat=0x%" PRIx32 " [%02X:%02X:%02X:%02X:%02X:%02X]",
                 param->conn_stat.state,
                 param->conn_stat.peer_feat,
                 param->conn_stat.chld_feat,
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_CONNECTED)
        {
            memcpy(connected_device, bda, 6);
            led_mode = 2; // 先认为链路建立，但还未完成SLC
            ESP_LOGI(TAG, "HFP RFCOMM已连接，等待SLC建立...");
        }
        else if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED)
        {
            hfp_connected = true;
            memcpy(connected_device, bda, 6);
            led_mode = 2; // 绿灯常亮

            ESP_LOGI(TAG, "✓ HFP SLC已建立");
        }
        else
        {
            if (dial_alerting_timer != NULL) xTimerStop(dial_alerting_timer, 0);
            hfp_connected = false;
            memset(connected_device, 0, 6);
            current_call_state = CALL_STATE_IDLE;
            led_mode = 1; // 蓝灯慢闪
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            ESP_LOGI(TAG, "HFP断开后恢复为可搜索状态，等待车机重新连接");

            // 停止RING
            // if (ring_timer != NULL)
            // {
            //     xTimerStop(ring_timer, 0);
            // }     
        }
        break;
    }

    case ESP_HF_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "HFP音频状态: %s",
                 (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED) ? "已连接" : "已断开");
        break;

    case ESP_HF_ATA_RESPONSE_EVT:
        // 车机按下了"接听"按钮
        ESP_LOGI(TAG, "🎯 车机发送接听命令");
        handle_call_answer();
        break;

    case ESP_HF_CHUP_RESPONSE_EVT:
        // 车机按下了"挂断/拒接"按钮
        ESP_LOGI(TAG, "🎯 车机发送挂断命令");
        if (dial_alerting_timer != NULL) xTimerStop(dial_alerting_timer, 0);
        if (current_call_state == CALL_STATE_INCOMING)
        {
            handle_call_reject();
        }
        else if (current_call_state == CALL_STATE_ACTIVE)
        {
            handle_call_hangup();
        }
        else if (current_call_state == CALL_STATE_DIALING ||
                 current_call_state == CALL_STATE_ALERTING)
        {
            const char *name = lookup_contact_name(current_phone_number);
            pbap_append_calllog(outgoing_calls, &outgoing_call_count, name, current_phone_number);
            current_call_state = CALL_STATE_IDLE;
            led_mode = 2;
            esp_hf_ag_end_call(connected_device, 0, 0,
                ESP_HF_CALL_STATUS_NO_CALLS,
                ESP_HF_CALL_SETUP_STATUS_IDLE,
                current_phone_number,
                ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
            sync_hfp_call_indicators(0, 0);
            esp_hf_ag_audio_disconnect(connected_device);
            memset(current_phone_number, 0, sizeof(current_phone_number));
            ESP_LOGI(TAG, "📵 车机取消外拨");
        }
        break;

    case ESP_HF_DIAL_EVT:
        // 车机发起拨号
        ESP_LOGI(TAG, "🎯 车机发起拨号");
        if (param->out_call.num_or_loc)
        {
            handle_call_dial(param->out_call.num_or_loc);
        }
        break;

    case ESP_HF_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "音量控制: type=%d, volume=%d",
                 param->volume_control.type, param->volume_control.volume);
        break;

    case ESP_HF_BVRA_RESPONSE_EVT:
        ESP_LOGI(TAG, "语音识别: %s",
                 param->vra_rep.value ? "启用" : "禁用");
        break;

    case ESP_HF_BCS_RESPONSE_EVT:
        negotiated_hfp_codec = param->bcs_rep.mode;
        ESP_LOGI(TAG, "HFP音频编解码协商结果(mode=%d)", param->bcs_rep.mode);
        break;

    case ESP_HF_WBS_RESPONSE_EVT:
        negotiated_hfp_codec = param->wbs_rep.codec;
        ESP_LOGI(TAG, "HFP宽带语音状态(codec=%d)", param->wbs_rep.codec);
        break;

    case ESP_HF_CIND_RESPONSE_EVT:
    {
        esp_hf_call_status_t call = ESP_HF_CALL_STATUS_NO_CALLS;
        esp_hf_call_setup_status_t callsetup = ESP_HF_CALL_SETUP_STATUS_IDLE;
        get_hfp_call_snapshot(&call, &callsetup);
        ESP_LOGI(TAG, "HF请求CIND，返回当前通话状态(call=%d, setup=%d)", call, callsetup);
        esp_hf_ag_cind_response(
            param->cind_rep.remote_addr,
            call,
            callsetup,
            ESP_HF_NETWORK_STATE_AVAILABLE,
            5,
            0,
            5,
            0);
        break;
    }

    case ESP_HF_COPS_RESPONSE_EVT:
        ESP_LOGI(TAG, "HF请求运营商信息");
        esp_hf_ag_cops_response(param->cops_rep.remote_addr, "CMCC");
        break;

    case ESP_HF_CNUM_RESPONSE_EVT:
        // esp_hf_ag_cnum_response 底层同样有 ok_flag bug，不会发 OK
        // 车机等 OK 超时 7 秒 → 认为无法获取本机号码 → 标记为"网络电话"
        // 用 unknown_at_send 手工拼完整 +CNUM 响应 + OK
        // 格式: +CNUM: ,"号码",type,,service
        ESP_LOGI(TAG, "HF请求本机号码");
        esp_hf_ag_unknown_at_send(param->cnum_rep.remote_addr,
            "+CNUM: ,\"" CNUM_PHONE_NUMBER "\",129,,4\r\n\r\nOK");
        break;

    case ESP_HF_CLCC_RESPONSE_EVT:
        ESP_LOGI(TAG, "HF请求当前通话列表");
        respond_current_calls(param->clcc_rep.remote_addr);
        break;

    case ESP_HF_UNAT_RESPONSE_EVT:
    {
        const char *unat = param->unat_rep.unat;
        ESP_LOGW(TAG, "收到未知AT命令: %s", unat ? unat : "(null)");

        // 车机通过 CGMI/CGMM/CGMR 判断是否为真实手机
        // 全部返回ERROR → 车机认定为"网络电话"设备
        //
        // ESP-IDF bug: esp_hf_ag_unknown_at_send 在BTC层没有把 ok_flag 设为
        // BTA_AG_OK_DONE，导致 bta_ag_send_ok 不会被调用。
        // 底层 bta_ag_send_result 输出: \r\n<str>\r\n
        // 所以我们把 \r\n\r\nOK 嵌到字符串尾部，让完整响应变成:
        //   \r\nSamsung\r\n\r\nOK\r\n  ← 标准AT响应格式
        if (unat != NULL && strstr(unat, "+CGMI") != NULL)
        {
            ESP_LOGI(TAG, "📱 回复手机厂商: Samsung");
            esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr,
                "Samsung\r\n\r\nOK");
        }
        else if (unat != NULL && strstr(unat, "+CGMM") != NULL)
        {
            ESP_LOGI(TAG, "📱 回复手机型号: SM-G998B");
            esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr,
                "SM-G998B\r\n\r\nOK");
        }
        else if (unat != NULL && strstr(unat, "+CGMR") != NULL)
        {
            ESP_LOGI(TAG, "📱 回复固件版本: G998BXXS9FXA1");
            esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr,
                "G998BXXS9FXA1\r\n\r\nOK");
        }
        else
        {
            esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr, NULL);
        }
        break;
    }

    default:
        ESP_LOGD(TAG, "HFP未处理事件: %d", event);
        break;
    }
}

/* ===================== GAP事件回调 ===================== */

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "✓ 配对成功");
        }
        else
        {
            ESP_LOGE(TAG, "✗ 配对失败: %d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
        ESP_LOGI(TAG, "收到PIN码请求，回复固定PIN: 1234");
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "收到配对确认请求: %" PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "收到配对密钥通知: %" PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "收到配对密钥输入请求");
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "GAP模式变化: %d", param->mode_chg.mode);
        break;

    default:
        ESP_LOGD(TAG, "GAP未处理事件: %d", event);
        break;
    }
}

/* ===================== PBAP 通讯录同步 ===================== */

// PBAP Target UUID (OBEX连接用)
static const uint8_t pbap_target_uuid[16] = {
    0x79,0x61,0x35,0xf0, 0xf0,0xc5,0x11,0xd8,
    0x09,0x66,0x08,0x00, 0x20,0x0c,0x9a,0x66
};

// SDP数据类型常量（来自 sdpdefs.h，此处本地定义避免头文件依赖）
#ifndef UINT_DESC_TYPE
#define UINT_DESC_TYPE  1
#endif

static uint32_t pbap_sdp_handle = 0;
static uint32_t pbap_spp_hdl = 0;
static uint8_t  pbap_scn = 0;
static uint16_t pbap_peer_mtu = 255;
static bool     pbap_inited = false;

// 延迟发送机制：esp_spp_write 不能在 SPP 回调(BTC任务)中直接调用，
// 否则会往同一个 BTC 队列递归投递消息导致 assert 崩溃。
// 解决：把待发数据存下来，用 FreeRTOS 定时器从定时器任务中发送。
static uint8_t  *pbap_tx_buf = NULL;
static uint16_t  pbap_tx_len = 0;
static uint32_t  pbap_tx_handle = 0;
static TimerHandle_t pbap_tx_timer = NULL;

static void pbap_tx_timer_cb(TimerHandle_t xTimer)
{
    if (pbap_tx_buf && pbap_tx_len > 0 && pbap_tx_handle) {
        esp_spp_write(pbap_tx_handle, pbap_tx_len, pbap_tx_buf);
        free(pbap_tx_buf);
        pbap_tx_buf = NULL;
        pbap_tx_len = 0;
        ESP_LOGI(TAG, "📒 PBAP 数据已发送");
    }
}

// 生成全部联系人的 vCard 数据（format: 0x00=v2.1, 0x01=v3.0）
static int build_phonebook_vcards(char *buf, int max_len, uint8_t format)
{
    int off = 0;
    bool use_vcard30 = (format == 0x01);
    for (size_t i = 0; i < sizeof(phonebook)/sizeof(phonebook[0]); i++) {
        int n = 0;
        if (use_vcard30) {
            // vCard 3.0 默认 UTF-8，避免携带部分车机不兼容的 CHARSET 参数
            n = snprintf(buf + off, max_len - off,
                "BEGIN:VCARD\r\n"
                "VERSION:3.0\r\n"
                "N:%s;;;;\r\n"
                "FN:%s\r\n"
                "TEL;TYPE=CELL:%s\r\n"
                "END:VCARD\r\n",
                phonebook[i].name, phonebook[i].name, phonebook[i].number);
        } else {
            n = snprintf(buf + off, max_len - off,
                "BEGIN:VCARD\r\n"
                "VERSION:2.1\r\n"
                "N;CHARSET=UTF-8:%s;;;;\r\n"
                "FN;CHARSET=UTF-8:%s\r\n"
                "TEL;TYPE=CELL:%s\r\n"
                "END:VCARD\r\n",
                phonebook[i].name, phonebook[i].name, phonebook[i].number);
        }
        if (n < 0 || off + n >= max_len) break;
        off += n;
    }
    return off;
}

static int build_calllog_vcards(char *buf, int max_len, uint8_t format,
                                const calllog_t *logs, size_t log_count,
                                const char *call_type)
{
    int off = 0;
    bool use_vcard30 = (format == 0x01);
    for (size_t i = 0; i < log_count; i++) {
        int n = 0;
        if (use_vcard30) {
            n = snprintf(buf + off, max_len - off,
                "BEGIN:VCARD\r\n"
                "VERSION:3.0\r\n"
                "N:%s;;;;\r\n"
                "FN:%s\r\n"
                "TEL;TYPE=CELL:%s\r\n"
                "X-IRMC-CALL-DATETIME:%s\r\n"
                "X-IRMC-CALL-DATETIME;TYPE=%s:%s\r\n"
                "END:VCARD\r\n",
                logs[i].name, logs[i].name, logs[i].number,
                logs[i].datetime, call_type, logs[i].datetime);
        } else {
            n = snprintf(buf + off, max_len - off,
                "BEGIN:VCARD\r\n"
                "VERSION:2.1\r\n"
                "N;CHARSET=UTF-8:%s;;;;\r\n"
                "FN;CHARSET=UTF-8:%s\r\n"
                "TEL;TYPE=CELL:%s\r\n"
                "X-IRMC-CALL-DATETIME:%s\r\n"
                "X-IRMC-CALL-DATETIME;TYPE=%s:%s\r\n"
                "END:VCARD\r\n",
                logs[i].name, logs[i].name, logs[i].number,
                logs[i].datetime, call_type, logs[i].datetime);
        }
        if (n < 0 || off + n >= max_len) break;
        off += n;
    }
    return off;
}

typedef enum {
    PB_OBJ_PHONEBOOK = 0,
    PB_OBJ_ICH,
    PB_OBJ_OCH,
    PB_OBJ_MCH,
    PB_OBJ_CCH,
} pb_object_t;

static bool utf16be_contains_ascii(const uint8_t *buf, uint16_t start, uint16_t end, const char *token)
{
    size_t tlen = strlen(token);
    if (tlen == 0) return false;
    for (uint16_t k = start; (uint32_t)k + (uint32_t)(tlen * 2) <= end; k += 2) {
        bool match = true;
        for (size_t i = 0; i < tlen; i++) {
            if (buf[k + i * 2] != 0 || buf[k + i * 2 + 1] != (uint8_t)token[i]) {
                match = false;
                break;
            }
        }
        if (match) return true;
    }
    return false;
}

// 生成 vCard-listing XML（车机浏览联系人列表时请求）
static int build_vcard_listing(char *buf, int max_len)
{
    int off = 0;
    off += snprintf(buf + off, max_len - off,
        "<?xml version=\"1.0\"?>\r\n"
        "<!DOCTYPE vcard-listing SYSTEM \"vcard-listing.dtd\">\r\n"
        "<vCard-listing version=\"1.0\">\r\n");
    for (size_t i = 0; i < sizeof(phonebook)/sizeof(phonebook[0]); i++) {
        off += snprintf(buf + off, max_len - off,
            "<card handle=\"%d.vcf\" name=\"%s\"/>\r\n",
            (int)(i + 1), phonebook[i].name);
    }
    off += snprintf(buf + off, max_len - off, "</vCard-listing>\r\n");
    return off;
}

// 发送 OBEX 响应（延迟到定时器任务中执行，避免在 BTC 回调中调 esp_spp_write）
static void obex_send(uint32_t handle, uint8_t code,
                      const uint8_t *payload, uint16_t payload_len)
{
    uint16_t total = 3 + payload_len;
    uint8_t *pkt = malloc(total);
    if (!pkt) return;
    pkt[0] = code;
    pkt[1] = (total >> 8) & 0xFF;
    pkt[2] = total & 0xFF;
    if (payload_len > 0) memcpy(pkt + 3, payload, payload_len);

    // 释放上一次未发完的数据（正常不会发生）
    if (pbap_tx_buf) free(pbap_tx_buf);
    pbap_tx_buf = pkt;      // 转移所有权，不在这里 free
    pbap_tx_len = total;
    pbap_tx_handle = handle;

    if (pbap_tx_timer == NULL) {
        pbap_tx_timer = xTimerCreate("pbap_tx", 1, pdFALSE, NULL, pbap_tx_timer_cb);
    }
    if (pbap_tx_timer) {
        xTimerStart(pbap_tx_timer, 0);
    }
}

// 发送 OBEX 响应，带 End-of-Body
static void obex_send_body(uint32_t handle, const char *body, int body_len)
{
    uint16_t hdr_len = 3 + body_len;  // End-of-Body header: id(1) + len(2) + data
    uint8_t *resp = malloc(hdr_len);
    if (!resp) return;
    int p = 0;
    resp[p++] = 0x49;  // End of Body
    resp[p++] = (hdr_len >> 8) & 0xFF;
    resp[p++] = hdr_len & 0xFF;
    memcpy(resp + p, body, body_len);
    obex_send(handle, 0xA0, resp, hdr_len);  // 0xA0 = SUCCESS
    free(resp);
}

// 处理 OBEX CONNECT
static void obex_handle_connect(uint32_t handle, const uint8_t *data, uint16_t len)
{
    if (len < 7) return;
    uint16_t remote_mtu = (data[5] << 8) | data[6];
    if (remote_mtu > 64) pbap_peer_mtu = remote_mtu;

    uint8_t resp[32];
    int p = 0;
    resp[p++] = 0x10;  // OBEX version 1.0
    resp[p++] = 0x00;  // flags
    uint16_t our_mtu = 1024;
    resp[p++] = (our_mtu >> 8) & 0xFF;
    resp[p++] = our_mtu & 0xFF;
    // Who header（必须与 Target 相同，车机据此确认是 PBAP）
    resp[p++] = 0x4A;  // Who
    uint16_t who_len = 3 + 16;
    resp[p++] = (who_len >> 8) & 0xFF;
    resp[p++] = who_len & 0xFF;
    memcpy(resp + p, pbap_target_uuid, 16); p += 16;
    // Connection ID
    resp[p++] = 0xCB;
    resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x01;

    obex_send(handle, 0xA0, resp, p);
    ESP_LOGI(TAG, "📒 PBAP OBEX 连接成功 (peer MTU=%d)", pbap_peer_mtu);
}

// 构建 PBAP Application Parameters 响应头（PhonebookSize + NewMissedCalls）
static int build_app_params_response(uint8_t *buf, uint16_t pb_size)
{
    int p = 0;
    // OBEX header: Application Parameters (0x4C), byte-sequence
    buf[p++] = 0x4C;
    // 长度占位，稍后填
    int len_pos = p; p += 2;
    // Tag 0x08: PhonebookSize (UINT16 big-endian)
    buf[p++] = 0x08;
    buf[p++] = 0x02;
    buf[p++] = (pb_size >> 8) & 0xFF;
    buf[p++] = pb_size & 0xFF;
    // Tag 0x09: NewMissedCalls (UINT8)
    buf[p++] = 0x09;
    buf[p++] = 0x01;
    buf[p++] = 0x00;
    // 填长度（含 header ID + length 本身）
    uint16_t total_hdr_len = p;
    buf[len_pos]     = (total_hdr_len >> 8) & 0xFF;
    buf[len_pos + 1] = total_hdr_len & 0xFF;
    return p;
}

// 处理 OBEX GET（车机请求通讯录数据）
static void obex_handle_get(uint32_t handle, const uint8_t *data, uint16_t len)
{
    uint16_t pos = 3;  // 跳过 opcode + length
    bool want_phonebook = false;
    bool want_listing = false;
    uint16_t max_list_count = 0xFFFF;  // 默认：全部下载
    bool has_max_list_count = false;
    uint8_t requested_format = 0x00;
    pb_object_t requested_obj = PB_OBJ_PHONEBOOK;

    // ===== 解析所有 OBEX headers =====
    while (pos + 1 <= len) {
        uint8_t hid = data[pos];
        uint8_t hi = hid >> 6;

        if (hi == 0 || hi == 1) {  // Unicode(0) 或 byte-seq(1)，2字节长度
            if (pos + 3 > len) break;
            uint16_t hlen = (data[pos+1] << 8) | data[pos+2];
            if (hlen < 3 || pos + hlen > len) break;

            if (hid == 0x42) {  // Type header
                const char *t = (const char *)data + pos + 3;
                ESP_LOGI(TAG, "📒 GET Type: %.*s", (int)(hlen - 3), t);
                if (strstr(t, "x-bt/phonebook"))     want_phonebook = true;
                if (strstr(t, "x-bt/vcard-listing")) want_listing = true;
                if (strstr(t, "x-bt/vcard"))         want_phonebook = true;
            }
            else if (hid == 0x01) {  // Name header (UTF-16BE)
                uint16_t nstart = pos + 3;
                uint16_t nend = pos + hlen;
                if (utf16be_contains_ascii(data, nstart, nend, "ich")) requested_obj = PB_OBJ_ICH;
                else if (utf16be_contains_ascii(data, nstart, nend, "och")) requested_obj = PB_OBJ_OCH;
                else if (utf16be_contains_ascii(data, nstart, nend, "mch")) requested_obj = PB_OBJ_MCH;
                else if (utf16be_contains_ascii(data, nstart, nend, "cch")) requested_obj = PB_OBJ_CCH;
                else if (utf16be_contains_ascii(data, nstart, nend, "pb"))  requested_obj = PB_OBJ_PHONEBOOK;

                want_phonebook = true;
            }
            else if (hid == 0x4C) {  // Application Parameters
                // 解析 tag-length-value
                uint16_t ap_end = pos + hlen;
                uint16_t ap = pos + 3;
                while (ap + 2 <= ap_end) {
                    uint8_t tag = data[ap];
                    uint8_t tlen = data[ap + 1];
                    if (ap + 2 + tlen > ap_end) break;
                    if (tag == 0x04 && tlen == 2) {  // MaxListCount
                        max_list_count = (data[ap+2] << 8) | data[ap+3];
                        has_max_list_count = true;
                        ESP_LOGI(TAG, "📒 GET MaxListCount=%d", max_list_count);
                    }
                    if (tag == 0x07 && tlen == 1) {  // Format
                        requested_format = data[ap + 2];
                        ESP_LOGI(TAG, "📒 GET Format=0x%02X", requested_format);
                    }
                    ap += 2 + tlen;
                }
            }
            pos += hlen;
        } else if (hi == 3) { pos += 5; }  // 4字节值 (Connection ID 等)
          else if (hi == 2) { pos += 2; }  // 1字节值
          else break;
    }

    if (!want_phonebook && !want_listing) {
        ESP_LOGW(TAG, "📒 GET 未识别类型，默认返回通讯录");
        want_phonebook = true;
    }

    const calllog_t *selected_logs = NULL;
    size_t selected_log_count = 0;
    const char *obj_label = "pb";
    const char *selected_call_type = "DIALED";
    switch (requested_obj) {
        case PB_OBJ_ICH:
            selected_logs = incoming_calls;
            selected_log_count = incoming_call_count;
            obj_label = "ich";
            selected_call_type = "RECEIVED";
            break;
        case PB_OBJ_OCH:
            selected_logs = outgoing_calls;
            selected_log_count = outgoing_call_count;
            obj_label = "och";
            selected_call_type = "DIALED";
            break;
        case PB_OBJ_MCH:
            selected_logs = missed_calls;
            selected_log_count = missed_call_count;
            obj_label = "mch";
            selected_call_type = "MISSED";
            break;
        case PB_OBJ_CCH:
            obj_label = "cch";
            break;
        case PB_OBJ_PHONEBOOK:
        default:
            obj_label = "pb";
            break;
    }

    uint16_t pb_count = 0;
    if (requested_obj == PB_OBJ_PHONEBOOK) {
        pb_count = (uint16_t)(sizeof(phonebook) / sizeof(phonebook[0]));
    } else if (requested_obj == PB_OBJ_CCH) {
        pb_count = (uint16_t)(incoming_call_count + outgoing_call_count + missed_call_count);
    } else {
        pb_count = (uint16_t)selected_log_count;
    }
    ESP_LOGI(TAG, "📒 PBAP 请求对象: %s", obj_label);

    // ===== MaxListCount == 0：车机只想知道有多少条，不要数据 =====
    if (has_max_list_count && max_list_count == 0) {
        uint8_t resp[16];
        int rlen = build_app_params_response(resp, pb_count);
        obex_send(handle, 0xA0, resp, rlen);
        ESP_LOGI(TAG, "📒 PBAP 回复通讯录大小: %d 条", pb_count);
        return;
    }

    // ===== 正常下载通讯录 =====
    if (want_phonebook) {
        char *vcards = malloc(2048);
        if (!vcards) { obex_send(handle, 0xD3, NULL, 0); return; }
        int vlen = 0;
        if (requested_obj == PB_OBJ_PHONEBOOK) {
            vlen = build_phonebook_vcards(vcards, 2048, requested_format);
        } else if (requested_obj == PB_OBJ_CCH) {
            int off = 0;
            off += build_calllog_vcards(vcards + off, 2048 - off, requested_format,
                                        incoming_calls, incoming_call_count, "RECEIVED");
            off += build_calllog_vcards(vcards + off, 2048 - off, requested_format,
                                        outgoing_calls, outgoing_call_count, "DIALED");
            off += build_calllog_vcards(vcards + off, 2048 - off, requested_format,
                                        missed_calls, missed_call_count, "MISSED");
            vlen = off;
        } else {
            vlen = build_calllog_vcards(vcards, 2048, requested_format,
                                        selected_logs, selected_log_count, selected_call_type);
        }

        // 构建响应：Type + App Params + End-of-Body（部分车机需要 Type 才会入库）
        const char *mime = (requested_format == 0x01) ? "x-bt/phonebook;version=3.0" : "x-bt/phonebook;version=2.1";
        uint16_t type_hdr_len = (uint16_t)(3 + strlen(mime) + 1);  // 含 '\0'
        uint8_t app_params[16];
        int ap_len = build_app_params_response(app_params, pb_count);

        uint16_t eob_hdr_len = 3 + vlen;  // End-of-Body: id(1)+len(2)+data
        uint16_t payload_len = type_hdr_len + ap_len + eob_hdr_len;
        uint8_t *payload = malloc(payload_len);
        if (!payload) { free(vcards); obex_send(handle, 0xD3, NULL, 0); return; }

        int p = 0;
        payload[p++] = 0x42;  // Type
        payload[p++] = (type_hdr_len >> 8) & 0xFF;
        payload[p++] = type_hdr_len & 0xFF;
        memcpy(payload + p, mime, strlen(mime) + 1); p += (int)strlen(mime) + 1;
        memcpy(payload + p, app_params, ap_len); p += ap_len;
        payload[p++] = 0x49;  // End of Body
        payload[p++] = (eob_hdr_len >> 8) & 0xFF;
        payload[p++] = eob_hdr_len & 0xFF;
        memcpy(payload + p, vcards, vlen); p += vlen;

        obex_send(handle, 0xA0, payload, p);
        free(payload);
        free(vcards);
        ESP_LOGI(TAG, "📒 PBAP 发送对象=%s (%d bytes, %d 条, vCard %s)",
                 obj_label, vlen, pb_count, requested_format == 0x01 ? "3.0" : "2.1");
    } else if (want_listing) {
        char *listing = malloc(1024);
        if (!listing) { obex_send(handle, 0xD3, NULL, 0); return; }
        int llen = build_vcard_listing(listing, 1024);
        obex_send_body(handle, listing, llen);
        free(listing);
        ESP_LOGI(TAG, "📒 PBAP 发送联系人列表");
    }
}

// 创建 PBAP PSE 的 SDP 记录（让车机能通过 SDP 搜索发现 PBAP 服务）
static void pbap_create_sdp(uint8_t scn)
{
    pbap_sdp_handle = SDP_CreateRecord();
    if (pbap_sdp_handle == 0) {
        ESP_LOGE(TAG, "📒 创建 PBAP SDP 记录失败");
        return;
    }

    // Service Class: PBAP PSE (0x112F)
    uint16_t svc_class = 0x112F;
    SDP_AddServiceClassIdList(pbap_sdp_handle, 1, &svc_class);

    // Protocol: L2CAP → RFCOMM(scn) → OBEX
    tSDP_PROTOCOL_ELEM proto[3];
    memset(proto, 0, sizeof(proto));
    proto[0].protocol_uuid = 0x0100;  // L2CAP
    proto[0].num_params = 0;
    proto[1].protocol_uuid = 0x0003;  // RFCOMM
    proto[1].num_params = 1;
    proto[1].params[0] = scn;
    proto[2].protocol_uuid = 0x0008;  // OBEX
    proto[2].num_params = 0;
    SDP_AddProtocolList(pbap_sdp_handle, 3, proto);

    // Profile: Phonebook Access v1.2
    SDP_AddProfileDescriptorList(pbap_sdp_handle, 0x1130, 0x0102);

    // Service Name (attribute 0x0100, text string type=4)
    const char *svc_name = "Phonebook Access PSE";
    SDP_AddAttribute(pbap_sdp_handle, 0x0100, 4, strlen(svc_name) + 1, (uint8_t *)svc_name);

    // Supported Repositories: Local Phonebook (bit 0)
    uint8_t repos = 0x01;
    SDP_AddAttribute(pbap_sdp_handle, 0x0314, UINT_DESC_TYPE, 1, &repos);

    ESP_LOGI(TAG, "📒 PBAP SDP 记录已创建 (SCN=%d, handle=0x%lx)",
             scn, (unsigned long)pbap_sdp_handle);
}

// SPP 回调（PBAP 通过 SPP 的 RFCOMM 通道传输 OBEX 数据）
static void pbap_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            // scn=0: 让协议栈自动分配通道号
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE,
                              0, "PBAP_PSE");
        } else {
            ESP_LOGE(TAG, "📒 SPP 初始化失败: %d", param->init.status);
        }
        break;

    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            pbap_scn = param->start.scn;
            pbap_create_sdp(pbap_scn);
            ESP_LOGI(TAG, "📒 PBAP 服务就绪 (RFCOMM SCN=%d)", pbap_scn);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        pbap_spp_hdl = param->srv_open.handle;
        ESP_LOGI(TAG, "📒 PBAP 客户端已连接 (handle=%" PRIu32 ")", pbap_spp_hdl);
        break;

    case ESP_SPP_DATA_IND_EVT:
    {
        uint8_t *d = param->data_ind.data;
        uint16_t l = param->data_ind.len;
        if (l < 1) break;
        ESP_LOGI(TAG, "📒 PBAP OBEX opcode=0x%02X len=%d", d[0], l);

        switch (d[0]) {
        case 0x80:          // CONNECT
            obex_handle_connect(param->data_ind.handle, d, l);
            break;
        case 0x03: case 0x83: // GET / GET Final
            obex_handle_get(param->data_ind.handle, d, l);
            break;
        case 0x81:          // DISCONNECT
        case 0x85:          // SETPATH（车机切换目录，直接返回成功）
            obex_send(param->data_ind.handle, 0xA0, NULL, 0);
            ESP_LOGI(TAG, "📒 PBAP %s", d[0]==0x81 ? "DISCONNECT" : "SETPATH→OK");
            break;
        default:
            obex_send(param->data_ind.handle, 0xC6, NULL, 0); // NOT ACCEPTABLE
            ESP_LOGW(TAG, "📒 PBAP 未知 OBEX 0x%02X", d[0]);
            break;
        }
        break;
    }

    case ESP_SPP_CLOSE_EVT:
        pbap_spp_hdl = 0;
        ESP_LOGI(TAG, "📒 PBAP 连接关闭");
        break;

    default:
        break;
    }
}

// PBAP 初始化
static esp_err_t pbap_init(void)
{
    esp_err_t ret = esp_spp_register_callback(pbap_spp_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "📒 SPP 回调注册失败: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
    };
    ret = esp_spp_enhanced_init(&spp_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "📒 SPP 初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    pbap_inited = true;
    ESP_LOGI(TAG, "📒 PBAP 模块初始化中...");
    return ESP_OK;
}

// PBAP 反初始化
static void pbap_deinit(void)
{
    if (!pbap_inited) return;
    if (pbap_tx_timer) { xTimerStop(pbap_tx_timer, 0); }
    if (pbap_tx_buf) { free(pbap_tx_buf); pbap_tx_buf = NULL; }
    if (pbap_sdp_handle) {
        SDP_DeleteRecord(pbap_sdp_handle);
        pbap_sdp_handle = 0;
    }
    esp_spp_deinit();
    pbap_inited = false;
    pbap_spp_hdl = 0;
    pbap_scn = 0;
}

/* ===================== 蓝牙初始化 ===================== */

static esp_err_t bt_init(void)
{
    esp_err_t ret;
    bool controller_enabled = false;
    bool bluedroid_inited = false;
    bool bluedroid_enabled = false;
    bool avrc_tg_inited = false;
    bool a2dp_inited = false;

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "蓝牙控制器初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "蓝牙控制器启用失败: %s", esp_err_to_name(ret));
        goto fail;
    }
    controller_enabled = true;

    ret = esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_HCI);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "设置SCO数据路径失败: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "✓ SCO数据路径已设置为HCI");
    }

    // 初始化Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid初始化失败: %s", esp_err_to_name(ret));
        goto fail;
    }
    bluedroid_inited = true;

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid启用失败: %s", esp_err_to_name(ret));
        goto fail;
    }
    bluedroid_enabled = true;

    // 注册GAP回调
    esp_bt_gap_register_callback(bt_gap_cb);

    ret = configure_bt_identity();
    if (ret != ESP_OK)
    {
        goto fail;
    }

#if ENABLE_MEDIA_PROFILES
    // 车机通常会把“手机”当作 A2DP Source + AVRCP Target + HFP AG 的组合设备看待。
    // 仅暴露 HFP AG 时，部分车机会因为缺少 AVDTP(PSM 25) 服务而主动断开。
    ret = esp_avrc_tg_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "AVRCP TG初始化失败: %s", esp_err_to_name(ret));
        goto fail;
    }
    esp_avrc_tg_register_callback(avrc_tg_callback);
    avrc_tg_inited = true;

    esp_a2d_register_callback(a2dp_source_callback);
    ret = esp_a2d_source_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "A2DP Source初始化失败: %s", esp_err_to_name(ret));
        goto fail;
    }
    a2dp_inited = true;
#else
    ESP_LOGW(TAG, "⚠️ 已关闭 A2DP/AVRCP 以节省内存（ENABLE_MEDIA_PROFILES=0）");
#endif

    // 初始化HFP AG
    ret = esp_hf_ag_register_callback(hfp_ag_callback);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "HFP AG回调注册失败: %s", esp_err_to_name(ret));
        goto fail;
    }

    ret = esp_hf_ag_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "HFP AG初始化失败: %s", esp_err_to_name(ret));
        goto fail;
    }

    // 初始化 PBAP（通讯录同步），失败不影响电话功能
    ret = pbap_init();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "⚠️ PBAP 初始化失败，通讯录同步不可用");
    }

    ESP_LOGI(TAG, "✓ 蓝牙手机模拟器初始化成功，设备名: %s", bt_name);
    ESP_LOGI(TAG, "ℹ️ 手机可配对但通常不会建立HFP连接；车机/耳机等HF设备才会连接HFP AG");
    return ESP_OK;

fail:
    if (a2dp_inited)
    {
        esp_a2d_source_deinit();
    }
    if (avrc_tg_inited)
    {
        esp_avrc_tg_deinit();
    }
    if (bluedroid_enabled)
    {
        esp_bluedroid_disable();
    }
    if (bluedroid_inited)
    {
        esp_bluedroid_deinit();
    }
    if (controller_enabled)
    {
        esp_bt_controller_disable();
    }
    bt_cleanup_partial_init();
    return ret;
}

static void bt_deinit(void)
{
    // 关闭PBAP
    pbap_deinit();

    // 关闭HFP AG
    esp_hf_ag_deinit();
#if ENABLE_MEDIA_PROFILES
    esp_a2d_source_deinit();
    esp_avrc_tg_deinit();
#endif

    // 关闭Bluedroid
    esp_bluedroid_disable();
    esp_bluedroid_deinit();

    // 关闭蓝牙控制器
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    hfp_connected = false;
    current_call_state = CALL_STATE_IDLE;

    ESP_LOGI(TAG, "蓝牙已关闭");
}

static void bt_cleanup_partial_init(void)
{
    esp_bt_controller_status_t status = esp_bt_controller_get_status();

    if (status == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        esp_bt_controller_disable();
        status = esp_bt_controller_get_status();
    }

    if (status == ESP_BT_CONTROLLER_STATUS_INITED)
    {
        esp_bt_controller_deinit();
    }

    bt_on = false;
    hfp_connected = false;
    a2dp_connected = false;
    avrcp_connected = false;
    negotiated_hfp_codec = -1;
    current_call_state = CALL_STATE_IDLE;
}

/* ===================== 按键任务 ===================== */

static void button_task(void *arg)
{
    int last_boot = 1;
    int last_call = 1;

    while (1)
    {
        int now_boot = gpio_get_level(BOOT_KEY);
        int now_call = gpio_get_level(CALL_KEY);

        // ========== BOOT按键 - 开关蓝牙 ==========
        if (last_boot == 1 && now_boot == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // 消抖

            if (gpio_get_level(BOOT_KEY) == 0)
            {
                if (!bt_on)
                {
                    esp_err_t ret = start_bt_phone();
                    if (ret != ESP_OK)
                    {
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        led_mode = 0;
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "👆 关闭蓝牙手机模拟器");
                    bt_deinit();
                    bt_on = false;
                    led_mode = 0;
                }

                // 等待按键释放
                while (gpio_get_level(BOOT_KEY) == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        // ========== CALL按键 - 模拟来电 ==========
        if (last_call == 1 && now_call == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // 消抖

            if (gpio_get_level(CALL_KEY) == 0)
            {
                ESP_LOGI(TAG, "👆 触发模拟来电");
                simulate_incoming_call("13800138000");

                // 等待按键释放
                while (gpio_get_level(CALL_KEY) == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        last_boot = now_boot;
        last_call = now_call;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void switch_monitor_task(void *arg)
{
    int last_left = -1;
    int last_right = -1;
    int left_max_seen_position = 0;
    int max_seen_position = 0;
    TickType_t left_ignore_until = 0;
    TickType_t ignore_until = 0;

    while (1)
    {
        int left = read_bcd(BCD1_1, BCD1_2, BCD1_4, BCD1_8);
        int right = read_bcd(BCD2_1, BCD2_2, BCD2_4, BCD2_8);
        TickType_t now = xTaskGetTickCount();

        if (left != last_left && now >= left_ignore_until)
        {
            ESP_LOGI(TAG, "旋钮1: %d", left);

            if (left_max_seen_position == 0 && left == 0)
            {
                // 初始态
            }
            else if (left >= 1 && left <= 3)
            {
                if (left > left_max_seen_position)
                {
                    left_max_seen_position = left;
                    ESP_LOGI(TAG, "[旋钮1] 当前最高挡位=%d", left_max_seen_position);
                }
            }
            else if (left_max_seen_position >= 1 && left == 0)
            {
                const char *dial_number = DEFAULT_DIAL_NUMBER;
                if (left_max_seen_position == 2)
                {
                    dial_number = "13501693774";
                }
                else if (left_max_seen_position >= 3)
                {
                    dial_number = "13600136000";
                }

                ESP_LOGI(TAG, "📲 [旋钮1] 触发模拟呼出: %s", dial_number);
                handle_call_dial(dial_number);
                left_max_seen_position = 0;
                left_ignore_until = now + pdMS_TO_TICKS(300);
            }
            else
            {
                if (left == 0)
                {
                    left_max_seen_position = 0;
                }
                else
                {
                    ESP_LOGI(TAG, "[旋钮1] 经过中间挡位%d，等待回到0触发最高挡位=%d", left, left_max_seen_position);
                }
            }

            last_left = left;
        }

        if (right != last_right && now >= ignore_until)
        {
            ESP_LOGI(TAG, "旋钮2: %d", right);

            if (max_seen_position == 0 && right == 0)
            {
                // 初始态
            }
            else if (right >= 1 && right <= 3)
            {
                if (right > max_seen_position)
                {
                    max_seen_position = right;
                    ESP_LOGI(TAG, "[旋钮2] 当前最高挡位=%d", max_seen_position);
                }
            }
            else if (max_seen_position == 1 && right == 0)
            {
                ESP_LOGI(TAG, "📞 [旋钮2] 触发模拟来电: %s", DEFAULT_DIAL_NUMBER);
                simulate_incoming_call(DEFAULT_DIAL_NUMBER);
                max_seen_position = 0;
                ignore_until = now + pdMS_TO_TICKS(300);
            }
            else if (max_seen_position == 2 && right == 0)
            {
                ESP_LOGI(TAG, "📞 [旋钮2] 触发接通");
                handle_call_answer();
                max_seen_position = 0;
                ignore_until = now + pdMS_TO_TICKS(300);
            }
            else if (max_seen_position >= 3 && right == 0)
            {
                ESP_LOGI(TAG, "📞 [旋钮2] 触发挂断/拒接");
                if (current_call_state == CALL_STATE_INCOMING)
                {
                    handle_call_reject();
                }
                else if (current_call_state == CALL_STATE_ACTIVE)
                {
                    handle_call_hangup();
                }
                else if (current_call_state == CALL_STATE_DIALING ||
                         current_call_state == CALL_STATE_ALERTING)
                {
                    if (dial_alerting_timer != NULL) xTimerStop(dial_alerting_timer, 0);
                    current_call_state = CALL_STATE_IDLE;
                    led_mode = 2;
                    esp_hf_ag_end_call(
                        connected_device,
                        0,
                        0,
                        ESP_HF_CALL_STATUS_NO_CALLS,
                        ESP_HF_CALL_SETUP_STATUS_IDLE,
                        current_phone_number,
                        ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
                    sync_hfp_call_indicators(0, 0);
                    esp_hf_ag_audio_disconnect(connected_device);
                    memset(current_phone_number, 0, sizeof(current_phone_number));
                    ESP_LOGI(TAG, "📵 外拨已取消");
                }
                else
                {
                    ESP_LOGW(TAG, "❌ 当前没有可挂断的呼叫");
                }
                max_seen_position = 0;
                ignore_until = now + pdMS_TO_TICKS(300);
            }
            else
            {
                if (right == 0)
                {
                    max_seen_position = 0;
                }
                else
                {
                    ESP_LOGI(TAG, "[旋钮2] 经过中间挡位%d，等待回到0触发最高挡位=%d", right, max_seen_position);
                }
            }

            last_right = right;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===================== 主函数 ===================== */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t ret_wifi = wifi_init_sta_ap();
    if (ret_wifi != ESP_OK)
    {
        ESP_LOGW(TAG, "⚠️ Wi-Fi AP+STA 初始化失败: %s", esp_err_to_name(ret_wifi));
    }

    // 读取MAC地址生成唯一标识
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    snprintf(my_mac_id, sizeof(my_mac_id), "%02X%02X", mac[4], mac[5]);
    snprintf(bt_name, sizeof(bt_name), "BT_Phone_%s", my_mac_id);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  蓝牙手机模拟器 (HFP AG) [%s]", my_mac_id);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "  设备名: %s", bt_name);
    ESP_LOGI(TAG, "  PIN码: 1234");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    // 初始化LED
    gpio_reset_pin(LED_R);
    gpio_reset_pin(LED_G);
    gpio_reset_pin(LED_B);
    gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);
    led_off();

    // 初始化按键
    gpio_config_t boot_conf = {
        .pin_bit_mask = (1ULL << BOOT_KEY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&boot_conf);

    gpio_config_t call_conf = {
        .pin_bit_mask = (1ULL << CALL_KEY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&call_conf);

    gpio_config_t bcd_conf = {
        .pin_bit_mask = (1ULL << BCD1_4) | (1ULL << BCD1_8) |
                        (1ULL << BCD2_1) | (1ULL << BCD2_2) |
                        (1ULL << BCD2_4) | (1ULL << BCD2_8),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&bcd_conf);

    gpio_set_direction(BCD1_1, GPIO_MODE_INPUT);
    gpio_set_direction(BCD1_2, GPIO_MODE_INPUT);

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "当前旋钮: 左=%d, 右=%d",
             read_bcd(BCD1_1, BCD1_2, BCD1_4, BCD1_8),
             read_bcd(BCD2_1, BCD2_2, BCD2_4, BCD2_8));

    // 创建任务
    xTaskCreate(led_task, "led", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button", 4096, NULL, 5, NULL);
    xTaskCreate(switch_monitor_task, "switch", 4096, NULL, 5, NULL);

    esp_err_t ret_bt = start_bt_phone();
    if (ret_bt != ESP_OK)
    {
        ESP_LOGW(TAG, "⚠️ 开机自动启动蓝牙失败，可按BOOT键重试");
    }

    ESP_LOGI(TAG, "💡 系统就绪");
    ESP_LOGI(TAG, "💡 上电后会自动启动蓝牙，BOOT键可手动重启");
    ESP_LOGI(TAG, "💡 同时启动Wi-Fi：连接家庭网络并广播热点 %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "💡 按CALL键 (GPIO23) 模拟来电");
    ESP_LOGI(TAG, "💡 旋钮1: 1→0呼出13800138000, 2→0呼出13501693774, 3→0呼出13600136000");
    ESP_LOGI(TAG, "💡 旋钮2: 1→0模拟来电, 2→0接通, 3→0挂断/拒接");
    ESP_LOGI(TAG, "");
}
