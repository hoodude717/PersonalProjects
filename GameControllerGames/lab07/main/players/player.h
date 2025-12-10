#ifndef PLAYER_H_
#define PLAYER_H_


#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"

#define PLAYER_SPRITES 3
#define PLAYER_BITS_PER_PIXEL 1
#define PLAYER_LENGTH 1024
#define PLAYER_W 32
#define PLAYER_H 32

typedef enum {
    PLAYER_1, //Fireboy
    PLAYER_2  //Watergirl
} player_num_t;



typedef struct {
    coord_t ground_y;
    coord_t x;
    coord_t y;
    uint16_t jumpState;
    uint16_t moveState;
    bool dead;
    bool jump;
    bool right; //left if 0 right if 1
    bool onBtn1;
    bool onBtn2;
    player_num_t playerNum;

} player_t ;

//Fireboy Sprite arrays
extern const color_t *player_fire_left[PLAYER_SPRITES];
extern const color_t *player_fire_right[PLAYER_SPRITES];

//Watergirl Sprite arrays
extern const color_t *player_water_left[PLAYER_SPRITES];
extern const color_t *player_water_right[PLAYER_SPRITES];


//Draw the bitmap of the player at the coords,
//x: xpostion
//y: y position
//player: player to draw
void drawPlayer(player_t *player);

//set the ground underneath the player 
void player_set_ground(player_t *player, coord_t y);

//Change the players y
void player_set_y(player_t *player, coord_t newY);
//Change the players X
void player_set_x(player_t *player, coord_t newX);

//Return the state of jumping they are in
uint16_t player_get_jump_state();


#endif //PLAYER_H

