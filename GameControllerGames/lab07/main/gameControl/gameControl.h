#ifndef GAMECONTROL_H
#define GAMECONTROL_H

#include "player.h"
#include "switches.h"

//initialize all the game parts
void game_init(player_t *player1, player_t *player2);
//Print all the stuff to the screen for the level.
void game_setup_level(player_t *player1, player_t *player2);

//Tick and activate all other ticks
void game_tick(player_t *player1, player_t *player2);

//Tick function that calls all the necessary state machines for movement and everything
void game_play_tick(player_t *player1, player_t *player2);

//function for when it is the end of the game.
void game_end_game();

//function for start screen
void game_start();

//Winning the game screen
void game_win();

#endif