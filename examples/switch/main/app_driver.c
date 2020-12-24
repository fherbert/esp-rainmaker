/* Switch demo implementation using button and RGB LED
   
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>

#include <iot_button.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h> 
#include <esp_log.h>

#include <app_reset.h>
#include <ws2812_led.h>
#include "app_priv.h"

static const char *TAG = "app_driver";

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          0
#define BUTTON_ACTIVE_LEVEL  0

/**
 * zero crossing input on GPIO19
 * for frequency and delay calc we need to trigger on rising and falling edges.
 * once we've initialised, then we can change the trigger to rising edge only.
 */
#define zerocrossgpio 19
static uint32_t frequency = 0; /* default to 50Hz */
static uint32_t zcdelay = 0; /* how long in ms to wait after zc before voltage is at zero */
static xQueueHandle zc_evt_queue = NULL;
static uint64_t zcstarttime1 = 0; // timestamp for the start of 1st the zc pulse
static uint64_t zcstarttime2 = 0; // timestamp for the start of 2nd the zc pulse
static uint64_t zcendtime1 = 0; // timestamp for the end of the 1st zc pulse
static uint64_t zcendtime2 = 0; // timestamp for the end of the 2nd zc pulse
static uint64_t zctimeout = 1000000; // 1 second timeout waiting for zero crossing

static void IRAM_ATTR zc_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(zc_evt_queue, &gpio_num, NULL);
}

/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    20
static bool g_power_state = !DEFAULT_POWER; // The default power state should be off

/* These values correspoind to H,S,V = 120,100,10 */
#define DEFAULT_RED     0
#define DEFAULT_GREEN   25
#define DEFAULT_BLUE    0

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

static void app_indicator_set(bool state)
{
    if (state) {
        ws2812_led_set_rgb(DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE);
    } else {
        ws2812_led_clear();
    }
}

static void app_indicator_init(void)
{
    /* Steps:
    1. Enable 
    u_int32_t start_time;
    */
    ws2812_led_init();
    app_indicator_set(g_power_state);
}

static void push_btn_cb(void *arg)
{
    bool new_state = !g_power_state;
    app_driver_set_state(new_state);
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
            esp_rmaker_bool(new_state));
}

static void set_power_state(bool target)
{
    gpio_set_level(OUTPUT_GPIO, target);
    app_indicator_set(target);
}

static void zc_init_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(zc_evt_queue, &io_num, portMAX_DELAY)) {
            
            // check pin level, 
            if (gpio_get_level(io_num) == 1)
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void zc_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(zc_evt_queue, &io_num, portMAX_DELAY)) {
            // check pin level, 
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void app_driver_init()
{
    uint64_t timeout = 0; // timestamp for timeout of freq detection
    bool error = false; // zc detect error 
    uint64_t period = 0; // ac period
    button_handle_t btn_handle = iot_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Register a callback for a button tap (short press) event */
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }

    /* Configure power */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_GPIO);
    /* Configure the output GPIO */
    gpio_config(&io_conf);
    /* Configure zero crossing */
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((uint64_t)1 << zerocrossgpio);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    /*
    // set the initial interrupt for zc deteector to trigger on rising and falling edge
    gpio_set_intr_type(zerocrossgpio, GPIO_INTR_ANYEDGE);
    //create a queue to handle gpio event from isr
    zc_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start zc init task
    xTaskCreate(zc_init_task, "zc_init_task", 2048, NULL, 10, NULL);
    //install gpio isr service
    // Not required as io_button already does this??
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific zc gpio pin
    gpio_isr_handler_add(zerocrossgpio, zc_isr_handler, (void*) zerocrossgpio);
    */
    ESP_LOGI(TAG, "Init - Waiting for frequency detection");
    // Wait until frequency is set or timeout occurs
    zcstarttime1 = esp_timer_get_time();
    timeout = zcstarttime1 + zctimeout;
    // Wait until we see a 0 on the zc input pin
    while((zcstarttime1 = esp_timer_get_time()) < timeout && gpio_get_level(zerocrossgpio) == 1 ) {}
    if (zcstarttime1 >= timeout) {
        ESP_LOGI(TAG, "!!! Timeout waiting for low on zc input pin");
        error = true;
    }
    // DETECT FIRST ZC PULSE
    // Now wait for a 1 on the zc input pin
    zcstarttime1 = esp_timer_get_time();
    timeout = zcstarttime1 + zctimeout;
    while((zcstarttime1 = esp_timer_get_time()) < timeout && gpio_get_level(zerocrossgpio) == 0 ) {}
    if (zcstarttime1 >= timeout) {
        ESP_LOGI(TAG, "!!! Timeout waiting for high on zc input pin start of first pulse");
        error = true;
    }
    // Now wait for a 0 on the zc input pin
    zcendtime1 = esp_timer_get_time();
    timeout = zcendtime1 + zctimeout;
    while((zcendtime1 = esp_timer_get_time()) < timeout && gpio_get_level(zerocrossgpio) == 1 ) {}
    if (zcendtime1 >= timeout) {
        ESP_LOGI(TAG, "!!! Timeout waiting for low on zc input pin end of first pulse");
        error = true;
    }
    // END OF DETECT FIRST ZC PULSE
    // DETECT START OF SECOND ZC PULSE
    // wait for a 1 on the zc input pin
    zcstarttime2 = esp_timer_get_time();
    timeout = zcstarttime2 + zctimeout;
    while((zcstarttime2 = esp_timer_get_time()) < timeout && gpio_get_level(zerocrossgpio) == 0 ) {}
    if (zcstarttime2 >= timeout) {
        ESP_LOGI(TAG, "!!! Timeout waiting for high on zc input pin start of second pulse");
        error = true;
    }
    // Now wait for a 0 on the zc input pin
    zcendtime2 = esp_timer_get_time();
    timeout = zcendtime2 + zctimeout;
    while((zcendtime2 = esp_timer_get_time()) < timeout && gpio_get_level(zerocrossgpio) == 1 ) {}
    if (zcendtime2 >= timeout) {
        ESP_LOGI(TAG, "!!! Timeout waiting for low on zc input pin end of second pulse");
        error = true;
    }
    // END OF DETECT SECOND ZC PULSE
    if (!error) {
        zcdelay = ((zcendtime1 - zcstarttime1) / 2) + ((zcendtime2 - zcstarttime2) /2 ) / 2; // average duration of half the zc pulse over 2 pulses
        // Calculate hte period by subtracting the 2nd pulse starttime from the first pulse startime
        // then multiple it by 2 since the AC signal is rectified
        period = (zcstarttime2 - zcstarttime1) * 2;
        if (period > 17000) {
            frequency = 50 *2;
        } else {
            frequency = 60 *2;
        }
        ESP_LOGI(TAG, "Period detected: %llu us", period);
        ESP_LOGI(TAG, "Frequency detected: %u Hz", frequency);
    }
    //xTaskCreate(zc_task, "zc_task", 2048, NULL, 10, NULL);
    //hook isr handler for specific zc gpio pin
    //gpio_isr_handler_add(zerocrossgpio, zc_isr_handler, (void*) zerocrossgpio);
    
    app_indicator_init();
}

int IRAM_ATTR app_driver_set_state(bool state)
{
    if(g_power_state != state) {
        g_power_state = state;
        set_power_state(g_power_state);
    }
    return ESP_OK;
}

bool app_driver_get_state(void)
{
    return g_power_state;
}
