#ifndef PLAYERJUMP_H_
#define PLAYERJUMP_H_

#include <stdint.h>
#include "player.h"
#include "lcd.h"





//Initialize the players jump to zero
void player_jump_init(player_t *player);

//state machine for player jumping
void player_jump_tick(player_t *player);

//Function to check the platform underneath the player. IF the player is ontop of a platform then change the state to idle. 
void player_jump_hit_ground(player_t * player, uint16_t x, uint16_t y);

#endif //PLAYERMOVEMENT_H
