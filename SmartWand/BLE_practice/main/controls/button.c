#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"


volatile bool button_pressed = false;
static const char *TAG = "BUTTON";
static TimerHandle_t debounce_timer;
static volatile bool button_event_pending = false;

// ISR: Called on button press (falling edge)
static void IRAM_ATTR button_isr_handler(void* arg) {
    // Start debounce timer (or reset if running)
    if (debounce_timer != NULL) {
        xTimerResetFromISR(debounce_timer, NULL);
    }
}

// Timer callback after debounce time has passed
static void debounce_timer_callback(TimerHandle_t xTimer) {
    // Re-check the pin to confirm it's still low (pressed)
    if (gpio_get_level(BUTTON_GPIO) == 0) {
        button_event_pending = true;
    }
}

void button_init() {
    // Configure GPIO as input with pull-up resistor
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    // Create debounce software timer
    debounce_timer = xTimerCreate(
        "debounce_timer",
        pdMS_TO_TICKS(DEBOUNCE_TIME_MS),
        pdFALSE,  // One-shot timer
        NULL,
        debounce_timer_callback
    );

    // Install GPIO ISR service and attach handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

}

bool button_pressed() {
    return button_event_pending;
}

void button_clear_event() {
    button_event_pending = false;
}