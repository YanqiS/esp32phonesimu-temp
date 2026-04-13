#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"

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

#define TAG "BT_PHONE"

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

// ========== 全局变量 ==========
static char my_mac_id[8];
static char bt_name[32];
static bool bt_on = false;
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

typedef struct
{
    const char *name;
    const char *number;
} contact_t;

static const contact_t phonebook[] = {
    {"张三", "13800138000"},
    {"李四", "13501693774"},
    {"王五", "13600136000"},
    {"赵六", "13700137000"},
};

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

static void respond_current_calls(esp_bd_addr_t remote_addr)
{
    if (!hfp_connected)
    {
        ESP_LOGW(TAG, "SLC未建立，跳过CLCC响应");
        return;
    }

    if (current_call_state == CALL_STATE_DIALING)
    {
        ESP_LOGI(TAG, "CLCC返回: 外拨中 %s", current_phone_number);
        esp_hf_ag_clcc_response(
            remote_addr,
            1,
            ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
            ESP_HF_CURRENT_CALL_STATUS_DIALING,
            ESP_HF_CURRENT_CALL_MODE_VOICE,
            ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        return;
    }

    if (current_call_state == CALL_STATE_ACTIVE)
    {
        ESP_LOGI(TAG, "CLCC返回: 通话中 %s", current_phone_number);
        esp_hf_ag_clcc_response(
            remote_addr,
            1,
            ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
            ESP_HF_CURRENT_CALL_STATUS_ACTIVE,
            ESP_HF_CURRENT_CALL_MODE_VOICE,
            ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        return;
    }

    if (current_call_state == CALL_STATE_ALERTING)
    {
        ESP_LOGI(TAG, "CLCC返回: 对方振铃 %s", current_phone_number);
        esp_hf_ag_clcc_response(
            remote_addr,
            1,
            ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
            ESP_HF_CURRENT_CALL_STATUS_ALERTING,
            ESP_HF_CURRENT_CALL_MODE_VOICE,
            ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        return;
    }

    if (current_call_state == CALL_STATE_INCOMING)
    {
        ESP_LOGI(TAG, "CLCC返回: 来电中 %s", current_phone_number);
        esp_hf_ag_clcc_response(
            remote_addr,
            1,
            ESP_HF_CURRENT_CALL_DIRECTION_INCOMING,
            ESP_HF_CURRENT_CALL_STATUS_INCOMING,
            ESP_HF_CURRENT_CALL_MODE_VOICE,
            ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        return;
    }

    ESP_LOGI(TAG, "当前没有活动呼叫，CLCC返回空列表");
}

// 定时器回调：外拨1秒后从DIALING切到ALERTING
static void dial_alerting_timer_callback(TimerHandle_t xTimer)
{
    if (current_call_state == CALL_STATE_DIALING && hfp_connected)
    {
        current_call_state = CALL_STATE_ALERTING;
        ESP_LOGI(TAG, "📞 对方振铃中...");
        esp_hf_ag_out_call(
            connected_device,
            0,
            0,
            ESP_HF_CALL_STATUS_NO_CALLS,
            ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING,
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    }
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
        .minor = 0,
        .service = ESP_BT_COD_SRVC_TELEPHONY,
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
    if (current_call_state != CALL_STATE_INCOMING)
    {
        ESP_LOGW(TAG, "❌ 当前无来电，无法拒接");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "❌ ========== 拒接来电 ==========");
    ESP_LOGI(TAG, "❌ 电话号码: %s", current_phone_number);
    ESP_LOGI(TAG, "❌ ===============================");

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

    // 模拟对方振铃
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (current_call_state == CALL_STATE_DIALING)
    {
        current_call_state = CALL_STATE_ALERTING;
        ESP_LOGI(TAG, "📞 对方振铃中...");
        esp_hf_ag_out_call(
            connected_device,
            0,
            0,
            ESP_HF_CALL_STATUS_NO_CALLS,
            ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING, // 3
            current_phone_number,
            ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
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
 
            if (bsir_ret != ESP_OK)
            {
                ESP_LOGW(TAG, "BSIR 通知失败: %s", esp_err_to_name(bsir_ret));
            }
            else
            {
                ESP_LOGI(TAG, "✓ 已通知车机支持 In-Band Ring Tone");
            }  
        }
        else
        {
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
        if (current_call_state == CALL_STATE_INCOMING)
        {
            handle_call_reject();
        }
        else if (current_call_state == CALL_STATE_ACTIVE)
        {
            handle_call_hangup();
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
        ESP_LOGI(TAG, "HF请求CIND，返回空闲设备状态");
            esp_hf_ag_cind_response(
            param->cind_rep.remote_addr,
            ESP_HF_CALL_STATUS_NO_CALLS,
            ESP_HF_CALL_SETUP_STATUS_IDLE,
            ESP_HF_NETWORK_STATE_AVAILABLE,
            5,
            0,
            5,
            0);
        break;

    case ESP_HF_COPS_RESPONSE_EVT:
        ESP_LOGI(TAG, "HF请求运营商信息");
        esp_hf_ag_cops_response(param->cops_rep.remote_addr, "ESP32 Phone");
        break;

    case ESP_HF_CNUM_RESPONSE_EVT:
        ESP_LOGI(TAG, "HF请求本机号码");
        esp_hf_ag_cnum_response(param->cnum_rep.remote_addr, DEFAULT_DIAL_NUMBER, 129, 0);
        break;

    case ESP_HF_CLCC_RESPONSE_EVT:
        ESP_LOGI(TAG, "HF请求当前通话列表");
        respond_current_calls(param->clcc_rep.remote_addr);
        break;

    case ESP_HF_UNAT_RESPONSE_EVT:
        ESP_LOGW(TAG, "收到未知AT命令: %s", param->unat_rep.unat ? param->unat_rep.unat : "(null)");
        esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr, NULL);
        break;

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
    // 关闭HFP AG
    esp_hf_ag_deinit();
    esp_a2d_source_deinit();
    esp_avrc_tg_deinit();

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
                else if (current_call_state == CALL_STATE_DIALING)
                {
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
    ESP_LOGI(TAG, "💡 按CALL键 (GPIO23) 模拟来电");
    ESP_LOGI(TAG, "💡 旋钮1: 1→0呼出13800138000, 2→0呼出13501693774, 3→0呼出13600136000");
    ESP_LOGI(TAG, "💡 旋钮2: 1→0模拟来电, 2→0接通, 3→0挂断/拒接");
    ESP_LOGI(TAG, "");
}
