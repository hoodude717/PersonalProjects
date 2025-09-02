#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h" // For xTimerCreate, xTimerIsTimerActive, xTimerStartFromISR
#include "freertos/queue.h"  // For xQueueSendFromISR
#include "driver/gpio.h"
#include "esp_log.h"



// volatile bool button_pressed = false;
static const char *TAG = "BUTTON";
static TimerHandle_t debounce_timer;
static volatile bool button_pressed_event = false;
static volatile bool button_released_event = false;
static bool isr_service_installed = false;

// Queue to handle the button events
// This queue can be used to send button events to tasks if needed
QueueHandle_t xButtonEventQueue;

static int last_stable_button_level = 1; // Start assuming released (high)


// ISR: Triggered on ANY edge (both rising and falling)
static void IRAM_ATTR button_isr_handler(void* arg) {
    // Stop the timer to restart debounce if another edge occurs quickly
    if (debounce_timer != NULL) {
        xTimerStopFromISR(debounce_timer, NULL);
    }
    // Start the timer to check button state after debounce
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTimerStartFromISR(debounce_timer, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



// Timer callback after debounce time has passed
static void debounce_timer_callback(TimerHandle_t xTimer) {
    int current_button_level = gpio_get_level(BUTTON_PIN);
    button_event_t event_type;

    // Check if the button state has changed and is now stable
    if (current_button_level != last_stable_button_level) {
        if (current_button_level == 1) { // Button is now pressed (active low)
            event_type = BUTTON_EVENT_PRESSED;
            // No log here, moved to task
        } else { // Button is now released (active low)
            event_type = BUTTON_EVENT_RELEASED;
            // No log here, moved to task
        }
        last_stable_button_level = current_button_level; // Update last stable state
        xQueueSend(xButtonEventQueue, &event_type, 0); // Send to queue (non-ISR context here)
    }
    // If state hasn't changed, it was just noise or edge re-triggered during debounce
}

// // Timer callback after debounce time has passed
// static void debounce_timer_callback(TimerHandle_t xTimer) {
//     button_event_t event_type;
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     if (button_get_level()) {
//         button_pressed_event = true;
//         button_released_event = false;
//         //ESP_LOGI(TAG, "Button pressed (debounced)");
//         event_type = BUTTON_EVENT_PRESSED;
//     } else {
//         button_released_event = true;
//         button_pressed_event = false;
//         //ESP_LOGI(TAG, "Button released (debounced)");
//         event_type = BUTTON_EVENT_RELEASED;
//     }

//     xQueueSendFromISR(xButtonEventQueue, &event_type, &xHigherPriorityTaskWoken);

//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }

void button_init() {
    // Configure GPIO as input with pull-up resistor
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, // Assuming external pulldown, or internal if needed
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Based on your description "pull down resistor"
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

    // Create the button event queue
    xButtonEventQueue = xQueueCreate(5, sizeof(button_event_t)); // Queue for 5 events
    if (xButtonEventQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create button event queue!");
        // Handle error appropriately, maybe panic or retry
    }

    // Install GPIO ISR service and attach handler
    if (!isr_service_installed) {
        gpio_install_isr_service(0); // Pass 0 for default flags
        isr_service_installed = true;
    }
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
}

//Return the level of the button
bool button_get_level(){
    return gpio_get_level(BUTTON_PIN);
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
