#ifndef PLAYERMOVEMENT_H_
#define PLAYERMOVEMENT_H_

#include <stdint.h>
#include "player.h"
#include "lcd.h"





//Initialize the players different stats
void player_movement_init(player_t *player);

//state machine for player movement
void player_movement_tick(player_t *player);

//Set player to stop state
void player_stop(player_t *player);

#endif //PLAYERMOVEMENT_H
