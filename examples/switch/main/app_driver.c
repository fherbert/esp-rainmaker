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
#include "driver/timer.h"

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
static uint64_t on_percent = 0; //percentage of period time to turn on
#define ESP_INTR_FLAG_DEFAULT 0
/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    20
// IGBT output
#define OUTPUT_IGBT    41
#define ZCDELAY_TIMER 0
#define DIM_TIMER 1

bool state = 0;

static void IRAM_ATTR zc_isr_handler(void* arg)
{
    // set delay time
    //timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, ZCDELAY_TIMER, zcdelay);
    timer_set_alarm_value(TIMER_GROUP_0, ZCDELAY_TIMER, zcdelay);
    // reset timer
    timer_set_counter_value(TIMER_GROUP_0, ZCDELAY_TIMER, 0);
    // enable alarm
    timer_set_alarm(TIMER_GROUP_0, ZCDELAY_TIMER, TIMER_ALARM_EN);
    timer_start(TIMER_GROUP_0, ZCDELAY_TIMER); // start the zc delay timer
    //ESP_ERROR_CHECK(esp_timer_start_once(zcdelay_timer, zcdelay));
    /*if (state) {
        //gpio_set_level(OUTPUT_IGBT, 0);
        GPIO.out1_w1tc.val = ((uint32_t)1 << (OUTPUT_IGBT - 32));
        state = 0;
    } else {
        //gpio_set_level(OUTPUT_IGBT, 1);
        GPIO.out1_w1ts.val = ((uint32_t)1 << (OUTPUT_IGBT - 32));
        state = 1;
    }*/
}

static void IRAM_ATTR timer_group0_isr(void* arg)
{
    // enter critical protect
    timer_spinlock_take(TIMER_GROUP_0);
    // which timer is this for?
    // int timer_idx = (int) arg;
    // Retrieve the interrupt status from the timer that reported the interrupt 
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    /* Clear the interrupt and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        // stop the zcdelay timer
        timer_pause(TIMER_GROUP_0, ZCDELAY_TIMER);
        // clear zcdelay timer interrupt status
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, ZCDELAY_TIMER);
        // turn on igbt
        GPIO.out1_w1ts.val = ((uint32_t)1 << (OUTPUT_IGBT-32));
        // on_us is calculated by multiplying the period by 1000000 (to get microseconds) by the percentage on time - this result in seconds
        // the multiply by 1000 to get microseconds
        //on_us = (1/frequency) * 1000000 * (on_percent/100)
        //We can factorise this down to:
        uint64_t on_us = (on_percent*10000)/frequency;
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, DIM_TIMER, on_us);
        // reset the dim timer counter
        timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, DIM_TIMER, 0);
        // start the dim timer
        timer_start(TIMER_GROUP_0, DIM_TIMER);
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, DIM_TIMER);
    } else if (timer_intr & TIMER_INTR_T1) {
        // stop the dim timer
        timer_pause(TIMER_GROUP_0, DIM_TIMER);
        // clear the interrupt
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, DIM_TIMER);
        // turn off igbt
        GPIO.out1_w1tc.val = ((uint32_t)1 << (OUTPUT_IGBT - 32));
    }
    timer_spinlock_give(TIMER_GROUP_0);
}

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

void detect_freq(void);

/*
* Initialise the selected timer of group 0
*
* timer_idx - the timer number to initialise
* timer_period_us - the interval in microseconds of alarm to set
*/
void init_timer(int timer_idx, int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,
            .counter_en = false,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = true,
            .divider = 80   /* 1 us per tick */
    };
    
    timer_init(TIMER_GROUP_0, timer_idx, &config);
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0);
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_period_us);
    timer_set_alarm(TIMER_GROUP_0, timer_idx, TIMER_ALARM_EN);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, &timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

}

void app_driver_init()
{
    button_handle_t btn_handle = iot_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Register a callback for a button tap (short press) event */
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }
    
    on_percent = 50; // hard code on time to 50%

    /* Configure power */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
		.intr_type = GPIO_INTR_DISABLE,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_GPIO);
    /* Configure the output GPIO */
    gpio_config(&io_conf);
    /* configure IGBT output */
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_IGBT);
    gpio_config(&io_conf);
    /* Configure zero crossing */
    //io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((uint64_t)1 << zerocrossgpio);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    detect_freq();
	
    // initialise timers
    init_timer(ZCDELAY_TIMER, zcdelay);
    init_timer(DIM_TIMER, on_percent); // this time gets set everytime the zcdelay timer expires.

    // trigger int on zc rising edge
    gpio_set_intr_type(zerocrossgpio, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(zerocrossgpio, zc_isr_handler, (void*) zerocrossgpio);
	
    app_indicator_init();
}

void detect_freq(void)
{
    uint64_t timeout = 0; // timestamp for timeout of freq detection
    bool error = false; // zc detect error 
    uint64_t period = 0; // ac period
    //uint64_t zcdelay = 0; //how long after postive trigger is zero crossing point
	uint64_t zcstarttime1 = 0; // timestamp for the start of 1st the zc pulse
	uint64_t zcstarttime2 = 0; // timestamp for the start of 2nd the zc pulse
    uint64_t zcendtime1 = 0; // timestamp for the end of the 1st zc pulse
	uint64_t zcendtime2 = 0; // timestamp for the end of the 2nd zc pulse
	uint64_t zctimeout = 1000000; // 1 second timeout waiting for zero crossing
    
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
        zcdelay = (zcendtime1 - zcstarttime1) / 2; 
        ESP_LOGI(TAG, "Period detected: %llu us", period);
        ESP_LOGI(TAG, "Frequency detected: %u Hz", frequency);
        ESP_LOGI(TAG, "Zero Crossing delay: %u us", zcdelay);
    }
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
