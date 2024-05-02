#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define TAG "DRV8835"

// モーターPWM用
#define MOTOR_PWM_RESOLUTION_HZ 1000000       // 1MHz, 1us per tick
#define MOTOR_PWM_PERIOD 100                // 100 tick, 100μs = 10kHzのPWM周期

// モーターPWM用
int groupID;
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t oper;
mcpwm_cmpr_handle_t comparator;
mcpwm_gen_handle_t generator;

// ボリューム用
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_chan_cfg_t adc1_config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,    // 12bit = 0xFFF = 4,095  0～4,095に変換されます。
};

void init() {
    ESP_LOGI(TAG, "Init(S)");

    groupID = 1;

    // AIN1初期化 PASE/ENABLEモードなので通常のIO
    gpio_reset_pin(CONFIG_AIN1);
    gpio_set_direction(CONFIG_AIN1, GPIO_MODE_OUTPUT);

    // AIN2初期化 PWMパルス信号
    timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = groupID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MOTOR_PWM_PERIOD,
    };
    esp_err_t ret = mcpwm_new_timer(&timer_config, &timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer error %d", ret);
        return;
    }

    ESP_LOGI(TAG, "init(2)");
    oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = groupID,
    };
    ret = mcpwm_new_operator(&operator_config, &oper);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_operator error %d", ret);
        return;
    }
    ret = mcpwm_operator_connect_timer(oper, timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_operator_connect_timer error %d", ret);
        return;
    }

    ESP_LOGI(TAG, "init(3)");
    comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ret = mcpwm_new_comparator(oper, &comparator_config, &comparator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_comparator error %d", ret);
        return;
    }

    ESP_LOGI(TAG, "init(4)");
    generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = CONFIG_AIN2,
    };
    ret = mcpwm_new_generator(oper, &generator_config, &generator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_generator error %d", ret);
        return;
    }
    if (generator == NULL) {
        ESP_LOGE(TAG, "mcpwm_new_generator error");
    }

    ESP_LOGI(TAG, "init(5)");
    ret = mcpwm_comparator_set_compare_value(comparator, (uint32_t)0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value error %d", ret);
        return;
    }

    ESP_LOGI(TAG, "init(6)");
    ret = mcpwm_generator_set_action_on_timer_event(generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_generator_set_actions_on_timer_event error %d", ret);
        return;
    }
    ESP_LOGI(TAG, "init(7)");
    ret = mcpwm_generator_set_action_on_compare_event(generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_generator_set_actions_on_compare_event error %d", ret);
        return;
    }
    
    ESP_LOGI(TAG, "init(8)");
    ret = mcpwm_timer_enable(timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_enable error %d", ret);
        return;
    }
    ESP_LOGI(TAG, "init(9)");
    ret = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_start_stop error %d", ret);
        return;
    }

    // ボリューム用ADコンバータ初期化
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &adc1_config));

    ESP_LOGI(TAG, "Init(E)");
}

void app_main(void)
{
    nvs_flash_init();     // Flash初期化  (お約束のようなもの)

    init();

    int maxval = (1 << 12) - 1;

    while(1) {
        int val;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &val));
        int motor = (int)((double)val / (double)maxval * 100.0);
 
        // int bitwidth = (int)adc1_config.bitwidth;
        // double d = val * 3.3 / (pow(2, bitwidth) - 1);
        // ESP_LOGI(TAG, "volume = %d(%lfv), motor=%d", val, d, motor);

        // モーター正転
        gpio_set_level(CONFIG_AIN1, 1);
        esp_err_t ret;
        ret = mcpwm_comparator_set_compare_value(comparator, motor);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value error %d", ret);
        }

        // 100ms待機
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}