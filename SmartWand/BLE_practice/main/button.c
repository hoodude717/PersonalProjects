#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "imu.h"



// volatile bool button_pressed = false;
static const char *TAG = "BUTTON";
static TimerHandle_t debounce_timer;
static volatile bool button_pressed_event = false;
static volatile bool button_released_event = false;
static bool isr_service_installed = false;

// ISR: Called on button press (falling edge)

static void IRAM_ATTR button_isr_handler(void* arg) {
    if (debounce_timer != NULL && xTimerIsTimerActive(debounce_timer) == pdFALSE) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(debounce_timer, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

// Timer callback after debounce time has passed
static void debounce_timer_callback(TimerHandle_t xTimer) {
    if (button_get_level()) {
        button_pressed_event = true;
        button_released_event = false;
        ESP_LOGI(TAG, "Button pressed (debounced)");
        imu_reset_angles();
    } else {
        button_released_event = true;
        button_pressed_event = false;
        ESP_LOGI(TAG, "Button released (debounced)");
    }
}

void button_init() {
    // Configure GPIO as input with pull-up resistor
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // Trigger on both edges
    };
    gpio_config(&io_conf);

    // Create debounce software timer
    debounce_timer = xTimerCreate(
        "debounce_timer",
        pdMS_TO_TICKS(BUTTON_DEBOUNCE_TIME),
        pdFALSE,  // One-shot timer
        NULL,
        debounce_timer_callback
    );

    // Install GPIO ISR service and attach handler
    if (!isr_service_installed) {
        gpio_install_isr_service(0);
        isr_service_installed = true;

    }
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

}

//Return the level of the button
bool button_get_level(){
    return gpio_get_level(BUTTON_PIN); //High == 1 Change to 0 High
}


//One shot only when the button is pressed
bool button_pressed() {
    if (button_pressed_event) {
        button_pressed_event = false;  // One-shot behavior
        return true;
    }
    return false;
}

//One shot only when the button is released
bool button_released() {
    if (button_released_event) {
        button_released_event = false;  // One-shot behavior
        return true;
    }
    return false;
}
