#include "platforms.h"
#include "config.h"
#include "elevator.h"
#include <stdio.h>


#define GROUND LCD_H - CONFIG_PLATFORM_H
#define BOX CONFIG_BOX_Y
#define PLATFORM_1 CONFIG_PLATFORM_Y_1
#define PLATFORM_2 CONFIG_PLATFORM_Y_2
#define PLAYER_W_4 (CONFIG_PLAYER_WIDTH / 4)
#define ELEVATOR 104

//retruns a number 0 - 2 which platform the player is on. Used for checking to see if the player needs to fall or stay grounded. 
void platforms_get_platform(player_t *player) {
    coord_t platform = player->ground_y;

    coord_t feet = player->y + CONFIG_PLAYER_HEIGHT;
    coord_t elevatorY = elevator_get_y();
    if ((platform <= ELEVATOR_MAX_Y) && (platform >= ELEVATOR_MIN_Y)) {
        platform = ELEVATOR;
        }
    

    //logic for checking which platform it is on. 
    switch(platform) {
        case GROUND:
            //check if player is above the box
            if((feet < BOX) && ((player->x + CONFIG_PLAYER_WIDTH) > CONFIG_BOX_X)) {
                player_set_ground(player, BOX);
            }
            break;
        case BOX:
            //Check if player is above platform 1 else if is ground
            if((feet < PLATFORM_1) && (player->x < CONFIG_PLATFORM_W)) {
                player_set_ground(player, PLATFORM_1);
            } else if(((player->x + CONFIG_PLAYER_WIDTH) < CONFIG_BOX_X)) {
                player_set_ground(player, GROUND);
            }
            break;
        case PLATFORM_1:


            //Check if player is back over the box or shold fall to ground
            if((feet < BOX) && ((player->x + CONFIG_PLAYER_WIDTH) > CONFIG_BOX_X)) {
                player_set_ground(player, BOX);
            } else if(!(feet < PLATFORM_2) && ((player->x + PLAYER_W_4) > CONFIG_PLATFORM_W)) {
                player_set_ground(player, GROUND);
            } else if((feet < elevatorY) && (player->x < ELEVATOR_W)) {
                player_set_ground(player, elevatorY);
            }
            break;
        case ELEVATOR:

            // if(player_get_jump_state(player) == IDLE_ST) {
            //     player_set_y(player, elevatorY-CONFIG_PLAYER_HEIGHT);
            // }
            if(((feet < PLATFORM_1) && (feet > PLATFORM_2)) && (player->x > ELEVATOR_W)) {
                player_set_ground(player, PLATFORM_1);
            } else if((feet < PLATFORM_2) && ((player->x+CONFIG_PLAYER_WIDTH) > ELEVATOR_W)) {
                player_set_ground(player, PLATFORM_2);
            }
            break;
        case PLATFORM_2: 


            if(player->x < ELEVATOR_W) {
                player_set_ground(player, elevatorY);
            }
            break;
        default:
            platform = ELEVATOR;
    }

}