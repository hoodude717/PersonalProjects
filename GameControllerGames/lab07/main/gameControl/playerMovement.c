#include "playerMovement.h"
#include "config.h"
#include "cursor.h"
#include "playerJump.h"
#include "platforms.h"
#include "obstacles.h"
#include <stdio.h>

//Position offsets
#define X_START 0
#define Y_START (LCD_H - CONFIG_PLAYER_HEIGHT - CONFIG_PLATFORM_H)
#define CURS_CENTER_MAX (LCD_W/2) + 30 //Cursor within 30 pixels of center of screen wont cause the person to move
#define CURS_CENTER_MIN (LCD_W/2) - 30

typedef enum {
    IDLE_ST,
    MOVING_LEFT_ST,
    MOVING_RIGHT_ST,
    DIED_ST
} player_movement_state;


//Initialize the players different stats
void player_movement_init(player_t *player){
    player->x = X_START;
    player->y = Y_START;
    player->dead = false;
    player->jump = false;
    player->right = true;
    player->ground_y = Y_START + CONFIG_PLAYER_HEIGHT;
    player->jumpState = IDLE_ST;
    player->moveState = IDLE_ST;
    

}

//state machine for player movement
void player_movement_tick(player_t *player) {
    uint16_t currentState = player->moveState;
    coord_t nx, ny;
    cursor_get_pos(&nx, &ny);
    player_jump_tick(player);
    platforms_get_platform(player);

    //MOORE LOGIC
    switch (currentState)
    {
    case IDLE_ST:
    //Check to see if cursor is on left or right then move player
        if((nx > CURS_CENTER_MAX) && ((player->x <= CONFIG_RIGHT_WALL) && (player->x >= CONFIG_LEFT_WALL))) {
            player->moveState = MOVING_RIGHT_ST;
        }
        else if((nx < CURS_CENTER_MIN) && ((player->x <= CONFIG_RIGHT_WALL) && (player->x >= CONFIG_LEFT_WALL))) {
            player->moveState = MOVING_LEFT_ST;
        }
        break;

    case MOVING_LEFT_ST:
    //Check to see if cursor is back in the middle or player position is out of the sides of screen if so then stop
        if(((nx < CURS_CENTER_MAX) && (nx > CURS_CENTER_MIN)) || ((player->x >= CONFIG_RIGHT_WALL) || (player->x <= CONFIG_LEFT_WALL))) {
           player->moveState = IDLE_ST;
        }

        break;
    case MOVING_RIGHT_ST:
        //Check to see if cursor is back in the middle or player position is out of the sides of screen if so then stop
        if(((nx < CURS_CENTER_MAX) && (nx > CURS_CENTER_MIN)) || ((player->x >= CONFIG_RIGHT_WALL) || (player->x <= CONFIG_LEFT_WALL))) {
            player->moveState = IDLE_ST;
        }
        break;

    case DIED_ST:
        
        break;
    default:
        player->moveState = IDLE_ST;
        break;
    }

    //MOORE ACTION
    switch (currentState)
    {
    case IDLE_ST:

        break;
    case MOVING_LEFT_ST:
        player->right = false;
        player->x -= CONFIG_PLAYER_DISTANCE_PER_TICK;
        //check to see if payer x is out of bounds if so reset to zero
        if(player->x <= CONFIG_LEFT_WALL) {
            player->x = CONFIG_LEFT_WALL;
        }
        break;

    case MOVING_RIGHT_ST:
    player->right = true;
        //drawPlayer(player);
        player->x += CONFIG_PLAYER_DISTANCE_PER_TICK;
        //check to see if payer x is out of bounds if so reset to zero
        if(player->x >= CONFIG_RIGHT_WALL) {
            player->x = CONFIG_RIGHT_WALL;
        }
        break;
    default:
        break;
    }


}

//Set player to stop state
void player_stop(player_t *player) {
    player->moveState = IDLE_ST;
}