#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "player.h"
#include "config.h"
#include "playerMovement.h"

//COnstants for hit boxes of FIre Water and Goo
#define FIRE_X1 CONFIG_FIRE_X1
#define FIRE_X2 (CONFIG_FIRE_X3 + CONFIG_TRIANGLE_OFFSET_2)
#define FIRE_Y CONFIG_FIRE_Y2
#define WATER_X1 CONFIG_WATER_X1
#define WATER_X2 (CONFIG_WATER_X3 + CONFIG_TRIANGLE_OFFSET_2)
#define WATER_Y CONFIG_WATER_Y2
#define GOO_X1 CONFIG_GOO_X1
#define GOO_X2 (CONFIG_GOO_X3 + CONFIG_TRIANGLE_OFFSET_2)
#define GOO_Y CONFIG_GOO_Y2

//Constantly checks to see if the players are running in to obstacles. 
void obstacles_tick(player_t *player1);

//Check if there is a collision with the player and fire
bool obstacles_collision_fire(player_t *player);
//Check if there is a collsiions with water
bool obstacles_collision_water(player_t *player);
//Check if there is collision with goo
bool obstacles_collision_goo(player_t *player);

//Check if player is hitting door
bool obstacles_collision_door(player_t *player);

//checking the conditions for game over and sending a flag
bool obstacles_game_over();

//Check to see if the players are in the spots to win.
bool obstacles_players_win(player_t *player1, player_t *player2);


#endif //OBSTACLES_H
