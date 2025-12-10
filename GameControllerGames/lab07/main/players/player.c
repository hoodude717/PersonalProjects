#include "playerFire.h"
#include "playerWater.h"
#include "player.h"
#include "config.h"
#include <stdio.h>


const color_t *player_fire_left[] = {
	player_fire_left0,
	//player_fire_left1,
};

const color_t *player_fire_right[] = {
	player_fire_right0,
	//player_fire_right1,
};


const color_t *player_water_left[] = {
	player_water_left0,
	//player_water_left1,

};

const color_t *player_water_right[] = {
	player_water_right0,
	//player_water_right1,

};

void drawPlayer(player_t *player) {

	if(player->playerNum == PLAYER_1) {
		if(player->right) {
			lcd_drawRGBBitmap(player->x, player->y, player_fire_right[0], PLAYER_W, PLAYER_H);
		} else {
			lcd_drawRGBBitmap(player->x, player->y, player_fire_left[0], PLAYER_W, PLAYER_H);
		}	
	}
	else if (player->playerNum == PLAYER_2) {
		if(player->right) {
			lcd_drawRGBBitmap(player->x, player->y, player_water_right[0], PLAYER_W, PLAYER_H);
		} else {
			lcd_drawRGBBitmap(player->x, player->y, player_water_left[0], PLAYER_W, PLAYER_H);
		}
	}

}


void player_set_ground(player_t *player, coord_t new_y) {
	player->ground_y = new_y;
}

//Change the players y
void player_set_y(player_t *player, coord_t newY) {
	player->y = newY;
}

//Return the state of jumping they are in
uint16_t player_get_jump_state(player_t *player) {
	return player->jumpState;
}

//Change the players X
void player_set_x(player_t *player, coord_t newX) {
	player->x = newX;
}