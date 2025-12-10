#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include "joy.h"

#define NUM_INIT_READS 10
//Defining my initial position
int_fast32_t init_pos[2][NUM_INIT_READS];
int_fast32_t cur_position[2];
//global ADC Handle
adc_oneshot_unit_handle_t adc1;







int32_t joy_init() {

    //Making adc unit and initial config
    
    adc_oneshot_unit_init_cfg_t initConfig1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&initConfig1, &adc1));

    //congif the ADC Channels
    adc_oneshot_chan_cfg_t config1 = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_CHANNEL_6, &config1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_CHANNEL_7, &config1));

    //Read and average the init pos
    uint16_t positionX = 0;
    uint16_t positionY = 0;
    for(int8_t i = 0; i<NUM_INIT_READS; i++) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, ADC_CHANNEL_6, &init_pos[0][i]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, ADC_CHANNEL_7, &init_pos[1][i]));
    positionX += init_pos[0][i];
    positionY += init_pos[1][i];
    }// Start playing the specified tone.
// tone: one of the enumerated tone types.
// freq: frequency of the tone in Hz.
    init_pos[0][0] = positionX / NUM_INIT_READS;
    init_pos[1][0] = positionY / NUM_INIT_READS;
    


    
    return 0;

}

int32_t joy_deinit() {
    //delete Adc if it isnt null
    if (adc1 != NULL) {
        ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1));
    }
    return 0;
}

void joy_get_displacement(int32_t *dcx, int32_t *dcy) {
    //read current position
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, ADC_CHANNEL_6, &cur_position[0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, ADC_CHANNEL_7, &cur_position[1]));

    // Find the displacement from initial
    *dcx = cur_position[0] - init_pos[0][0];
    *dcy = cur_position[1] - init_pos[1][0];




}