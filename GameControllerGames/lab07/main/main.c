#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "lcd.h"
#include "players/player.h"
#include "hw.h"
#include "cursor.h"
#include "sound.h"
#include "pin.h"
#include "config.h"
#include "playerMovement.h"
#include "playerJump.h"
#include "gameControl.h"
#include "switches.h"
#include "elevator.h"
#include "door.h"
#include "jumpSound.h"





// The update period as an integer in ms
#define PER_MS ((uint32_t)(CONFIG_GAME_TIMER_PERIOD*1000))
#define TIME_OUT 500 // ms
 // Cursor size (width & height) in pixels


#define delayMS(ms) \
	vTaskDelay(((ms)+(portTICK_PERIOD_MS-1))/portTICK_PERIOD_MS)



// Main display constants
#define TITLE_CLR GREEN
#define STATUS_CLR WHITE
#define PAC_CLR YELLOW
#define STR_BUF_LEN 12 // string buffer length
#define FONT_SIZE 2
#define FONT_W (LCD_CHAR_W*FONT_SIZE)
#define FONT_H (LCD_CHAR_H*FONT_SIZE)
#define STATUS_W (FONT_W*3)

#define WAIT 2000 // milliseconds
#define DELAY_EX3 20 // milliseconds

// Object position and movement
#define OBJ_X 100
#define OBJ_Y 100
#define OBJ_MOVE 2 // pixels



static const char *TAG = "lab07";

TimerHandle_t update_timer; // Declare timer handle for update callback

player_t fireboy;

player_t watergirl;

volatile bool interrupt_flag;

uint32_t isr_triggered_count;
uint32_t isr_handled_count;

// Interrupt handler for game - use flag method
void update() {
	interrupt_flag = true;
	isr_triggered_count++;
}



void app_main(void)
{

	// ISR flag and counts
	interrupt_flag = false;
	isr_triggered_count = 0;
	isr_handled_count = 0;
	// Initialization
	lcd_init();
	lcd_frameEnable();	
	sound_init(JUMP_SOUND_SAMPLE_RATE);
	lcd_fillScreen(CONFIG_COLOR_BACKGROUND);
	lcd_setFontSize(FONT_SIZE);
	cursor_init(PER_MS);

	fireboy.playerNum = PLAYER_1;
	watergirl.playerNum = PLAYER_2;

	ESP_LOGI(TAG, "Start up");
	lcd_drawString(0, 0, "Fireboy and Watergirl", TITLE_CLR);


	// Configure I/O pins for buttons
	pin_reset(HW_BTN_A);
	pin_input(HW_BTN_A, true);
	pin_reset(HW_BTN_B);
	pin_input(HW_BTN_B, true);
	pin_reset(HW_BTN_MENU);
	pin_input(HW_BTN_MENU, true);
	pin_reset(HW_BTN_OPTION);
	pin_input(HW_BTN_OPTION, true);
	pin_reset(HW_BTN_SELECT);
	pin_input(HW_BTN_SELECT, true);
	pin_reset(HW_BTN_START);
	pin_input(HW_BTN_START, true);

	// Initialize update timer
	update_timer = xTimerCreate(
		"update_timer",        // Text name for the timer.
		pdMS_TO_TICKS(PER_MS), // The timer period in ticks.
		pdTRUE,                // Auto-reload the timer when it expires.
		NULL,                  // No need for a timer ID.
		update                 // Function called when timer expires.
	);
	if (update_timer == NULL) {
		ESP_LOGE(TAG, "Error creating update timer");
		return;
	}
	if (xTimerStart(update_timer, pdMS_TO_TICKS(TIME_OUT)) != pdPASS) {
		ESP_LOGE(TAG, "Error starting update timer");
		return;
	}

	//initialize game
	game_init(&fireboy, &watergirl);



	// Main game loop
	uint64_t t1, t2, tmax = 0; // For hardware timer values
	coord_t x, y; // For cursor position


	while (pin_get_level(HW_BTN_MENU)) // while MENU button not pressed *LOOP FUNCTION
	{
		while (!interrupt_flag) ;
		t1 = esp_timer_get_time();
		interrupt_flag = false;
		isr_handled_count++;
		game_tick(&fireboy, &watergirl);


		cursor_tick_2();
		// printf("Heap at point 2: %d\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

		cursor_get_pos(&x, &y);
		t2 = esp_timer_get_time() - t1;
		if (t2 > tmax) tmax = t2;
	}
	printf("Handled %lu of %lu interrupts\n", isr_handled_count, isr_triggered_count);
	printf("WCET us:%llu\n", tmax);
	sound_deinit();

}