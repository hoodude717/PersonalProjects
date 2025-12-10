#include <stdio.h>
#include "playerJump.h"
#include "config.h"
#include "cursor.h"
#include "pin.h"
#include "platforms.h"
#include "sound.h"
#include "jumpSound.h"


//Constant limits and speeds
#define JUMP_H 75
#define JUMP_SPEED CONFIG_PLAYER_DISTANCE_PER_TICK

typedef enum {
    IDLE_ST,
    JUMP_ST,
    FALL_ST
} jump_states;

//Initialize the players jump to zero
void player_jump_init(player_t *player) {
    player->jump = false;
}

//state machine for player jumping
void player_jump_tick(player_t *player) {
    uint16_t currentState = player->jumpState;
    
    //static coord_t tempY = LCD_H - JUMP_H;
    //coord_t tempY = player->ground_y;
    //platforms_get_platform(player);


    //MOORE TRANSTIONS LOGIC
    switch (currentState) {
        case IDLE_ST:
        // IF Button A is pressed move to jump state
            if(!pin_get_level(HW_BTN_A)) {
                sound_start(jump_sound, JUMP_SOUND_SAMPLES, false);
                player->jumpState= JUMP_ST;
            }
            //tempY = player->y + CONFIG_PLAYER_HEIGHT;
            // Add transition for if there is nothing beneath you
            else if((player->y + CONFIG_PLAYER_HEIGHT) < player->ground_y) {
                player->jumpState = FALL_ST;
            }

            break;

        case JUMP_ST:
        // See if player y is jump height
            if(player->y <= player->ground_y - JUMP_H) {
                player->jumpState = FALL_ST;
            }
        break;
        
        case FALL_ST:
            //See if player hits the ground
            if(((player->y + CONFIG_PLAYER_HEIGHT) >= player->ground_y)) {
                player->jumpState = IDLE_ST;
            }
            //check to see if player is hit platform
        break;
        
        default: 
            player->jumpState = IDLE_ST;

    }

    //MOORE ACTIONS
    switch (currentState) {
        case IDLE_ST:
            break;

        case JUMP_ST:
            player->jump = true;
            player->y -= JUMP_SPEED;

        break;
        
        case FALL_ST:
            player->jump = false;
            player->y += JUMP_SPEED;
            //Out of bounds check
            if((player->y + CONFIG_PLAYER_HEIGHT) >= player->ground_y) {
                player->y = player->ground_y - CONFIG_PLAYER_HEIGHT;
            }   
        break;
        
        default: 
            currentState = IDLE_ST;

    }


}


//Function to check the platform underneath the player. IF the player is ontop of a platform then change the state to idle. 
// void player_jump_hit_ground(player_t * player, uint16_t x, uint16_t y) {
//     if(player->y >= y) {   
//         player->state = IDLE_ST;

//     }
// }
