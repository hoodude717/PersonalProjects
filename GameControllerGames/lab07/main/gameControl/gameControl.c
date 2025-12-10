#include "gameControl.h"
#include "config.h"
#include "pin.h"
#include "playerMovement.h"
#include "elevator.h"
#include "door.h"
#include "obstacles.h"

uint16_t playerMoving = PLAYER_1;

enum game_state_e {START_ST, PLAY_ST, GAME_OVER_ST, WIN_ST};

//Print all the stuff to the screen for the level.
void game_setup_level(player_t *player1, player_t *player2) {
    lcd_fillScreen(CONFIG_COLOR_BACKGROUND);
    lcd_drawString(0, 0, "Level 1", CONFIG_TITLE_COLOR);
    lcd_fillRect(CONFIG_PLATFORM_X_LEFT, CONFIG_GROUND_Y, LCD_W, CONFIG_PLATFORM_H, CONFIG_COLOR_PLATFORM);
    lcd_fillRect(CONFIG_PLATFORM_X_LEFT, CONFIG_PLATFORM_Y_1, CONFIG_PLATFORM_W, CONFIG_PLATFORM_H, CONFIG_COLOR_PLATFORM);
    lcd_fillRect(CONFIG_PLATFORM_X_RIGHT, CONFIG_PLATFORM_Y_2, CONFIG_PLATFORM_W_2, CONFIG_PLATFORM_H, CONFIG_COLOR_PLATFORM);
    lcd_fillRect(CONFIG_BOX_X, CONFIG_BOX_Y, CONFIG_BOX_W, CONFIG_BOX_H, CONFIG_COLOR_PLATFORM);
    lcd_fillTriangle(CONFIG_FIRE_X1, CONFIG_FIRE_Y1, CONFIG_FIRE_X2, CONFIG_FIRE_Y2, CONFIG_FIRE_X3, CONFIG_FIRE_Y3, CONFIG_COLOR_FIRE);
    lcd_fillTriangle(CONFIG_WATER_X1, CONFIG_WATER_Y1, CONFIG_WATER_X2, CONFIG_WATER_Y2, CONFIG_WATER_X3, CONFIG_WATER_Y3, CONFIG_COLOR_WATER);
    lcd_fillTriangle(CONFIG_GOO_X1, CONFIG_GOO_Y1, CONFIG_GOO_X2, CONFIG_GOO_Y2, CONFIG_GOO_X3, CONFIG_GOO_Y3, CONFIG_COLOR_GOO);
    lcd_fillTriangle(CONFIG_FIRE_X1+CONFIG_TRIANGLE_OFFSET_1, CONFIG_FIRE_Y1, CONFIG_FIRE_X2+CONFIG_TRIANGLE_OFFSET_1, CONFIG_FIRE_Y2, CONFIG_FIRE_X3+CONFIG_TRIANGLE_OFFSET_1, CONFIG_FIRE_Y3, CONFIG_COLOR_FIRE);
    lcd_fillTriangle(CONFIG_WATER_X1+CONFIG_TRIANGLE_OFFSET_1, CONFIG_WATER_Y1, CONFIG_WATER_X2+CONFIG_TRIANGLE_OFFSET_1, CONFIG_WATER_Y2, CONFIG_WATER_X3+CONFIG_TRIANGLE_OFFSET_1, CONFIG_WATER_Y3, CONFIG_COLOR_WATER);
    lcd_fillTriangle(CONFIG_GOO_X1+CONFIG_TRIANGLE_OFFSET_1, CONFIG_GOO_Y1, CONFIG_GOO_X2+CONFIG_TRIANGLE_OFFSET_1, CONFIG_GOO_Y2, CONFIG_GOO_X3+CONFIG_TRIANGLE_OFFSET_1, CONFIG_GOO_Y3, CONFIG_COLOR_GOO);
    lcd_fillTriangle(CONFIG_FIRE_X1+CONFIG_TRIANGLE_OFFSET_2, CONFIG_FIRE_Y1, CONFIG_FIRE_X2+CONFIG_TRIANGLE_OFFSET_2, CONFIG_FIRE_Y2, CONFIG_FIRE_X3+CONFIG_TRIANGLE_OFFSET_2, CONFIG_FIRE_Y3, CONFIG_COLOR_FIRE);
    lcd_fillTriangle(CONFIG_WATER_X1+CONFIG_TRIANGLE_OFFSET_2, CONFIG_WATER_Y1, CONFIG_WATER_X2+CONFIG_TRIANGLE_OFFSET_2, CONFIG_WATER_Y2, CONFIG_WATER_X3+CONFIG_TRIANGLE_OFFSET_2, CONFIG_WATER_Y3, CONFIG_COLOR_WATER);
    lcd_fillTriangle(CONFIG_GOO_X1+CONFIG_TRIANGLE_OFFSET_2, CONFIG_GOO_Y1, CONFIG_GOO_X2+CONFIG_TRIANGLE_OFFSET_2, CONFIG_GOO_Y2, CONFIG_GOO_X3+CONFIG_TRIANGLE_OFFSET_2, CONFIG_GOO_Y3, CONFIG_COLOR_GOO);
    lcd_fillRect(CONFIG_FINAL_DOOR_X_FIRE, CONFIG_FINAL_DOOR_Y_FIRE, CONFIG_FINAL_DOOR_W, CONFIG_FINAL_DOOR_H, CONFIG_COLOR_FIRE);
    lcd_fillRect(CONFIG_FINAL_DOOR_X_WATER, CONFIG_FINAL_DOOR_Y_WATER, CONFIG_FINAL_DOOR_W, CONFIG_FINAL_DOOR_H, CONFIG_COLOR_WATER);

    trigger_draw();    
    elevator_draw();
    door_draw();
    drawPlayer(player1);
    drawPlayer(player2);
    lcd_writeFrame();
}


//Tick and activate all other ticks
void game_tick(player_t *player1, player_t *player2) {
    static uint8_t currentState = START_ST;

    switch (currentState)
    {
    case START_ST:
        if(!pin_get_level(HW_BTN_START)) {
            currentState = PLAY_ST;
        }
        game_start();
        break;
    case PLAY_ST:
        if(obstacles_game_over()) {
            currentState = GAME_OVER_ST;
        }        
        //ADD THE GAME WON STATE
        else if(obstacles_players_win(player1, player2)) {
            currentState = WIN_ST;
        }

        game_setup_level(player1, player2);
        game_play_tick(player1, player2);

        break;
    case GAME_OVER_ST:
        if(!pin_get_level(HW_BTN_START)) {
            currentState = PLAY_ST;
        }
        game_init(player1, player2);
        game_end_game();
        break;

    case WIN_ST:
        if(!pin_get_level(HW_BTN_START)) {
            currentState = PLAY_ST;
        }
        game_init(player1, player2);
        game_win();
        
        break;
    default:
        break;
    }


}

//Tick function that calls all the necessary state machines for movement and everything
void game_play_tick(player_t *player1, player_t *player2) {
    static bool pressed = false;
    //One Shot to check for player switch
    if(!pin_get_level(HW_BTN_OPTION) && !pressed) {
        playerMoving = ~playerMoving;
        pressed = true;
    } else if (pin_get_level(HW_BTN_OPTION) && pressed) {
        pressed = false;
    }
    switches_tick(player1, player2);
    elevator_tick();
    door_tick(player1, player2);

    //check to see which player is being moved and call the move tick for that player.
    if(playerMoving == PLAYER_1) {
        //player_movement_tick(player1);
        obstacles_tick(player1);
    } else {
        //player_movement_tick(player2);
        obstacles_tick(player2);
    }
}

//function for when it is the end of the game.
void game_end_game() {
    lcd_fillScreen(CONFIG_GAME_OVER_COLOR);
    lcd_drawString(0, 0, "YOU DIED", RED);
    lcd_drawString(0, CONFIG_TEXT_H, "PRESS START TO RESTART", RED);

    lcd_writeFrame();
}

//function for start screen
void game_start() {
    lcd_fillScreen(CONFIG_GAME_OVER_COLOR);
    lcd_drawString(0, 0, "FIREBOY", RED);
    lcd_drawString(112, 0, " AND ", YELLOW);
    lcd_drawString(192, 0, "WATERGIRL", BLUE);
    lcd_drawString(0, LCD_H/2, "PRESS START TO START", BLUE);    

    lcd_writeFrame();
}

//initialize all the game parts
void game_init(player_t *player1, player_t *player2) {
    player_movement_init(player1);
    player_movement_init(player2);
    switches_init();
	elevator_init();
	door_init();
        playerMoving = PLAYER_1;
}


//Winning the game screen
void game_win() {
    lcd_fillScreen(CONFIG_GAME_OVER_COLOR);
    lcd_drawString(0, 0, "YOU WON!", YELLOW);  
    lcd_drawString(0, CONFIG_TEXT_H, "PRESS START TO RESTART", YELLOW);  

    lcd_writeFrame();
}