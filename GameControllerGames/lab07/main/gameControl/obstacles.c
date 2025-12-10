#include "obstacles.h"
#include "door.h"
#include <stdio.h>




//flag for game over
bool game_over_flag = false;
//Constantly checks to see if the players are running in to obstacles. 
void obstacles_tick(player_t *player1) {
    if (((player1->x + CONFIG_PLAYER_WIDTH) >= CONFIG_BOX_X) && ((player1->y+CONFIG_PLAYER_HEIGHT) > CONFIG_BOX_Y)) {
        //player_stop(player1);
        player_set_x(player1, CONFIG_BOX_X-CONFIG_PLAYER_WIDTH-1);
    } else if (obstacles_collision_door(player1)) {
        //player_stop(player1);
        if(player1->right){
            player_set_x(player1, player1->x - 5);
        } else {
           player_set_x(player1, player1->x + 5); 
        }
    } else {
        player_movement_tick(player1);
    }
    
    //Checking for end of game condition
    if (player1->playerNum) {
        //Collision checking
        //if watergirl
        if(obstacles_collision_fire(player1) || obstacles_collision_goo(player1)) {
            //printf("GAME_OVER!");
            game_over_flag = true;
        } else {
            game_over_flag = false;
        }
    } else {
        //if fireboy
        if(obstacles_collision_water(player1) || obstacles_collision_goo(player1)) {
            //printf("GAME_OVER!");
            game_over_flag = true;
        } else {
            game_over_flag = false;
        }

    } 
}

//Check if there is a collision with the player and fire
bool obstacles_collision_fire(player_t *player) {
    if(!player->playerNum) {
        return false;
    }
    if((player->x < FIRE_X2) && ((player->x+CONFIG_PLAYER_WIDTH) > FIRE_X1) && ((player->y+CONFIG_PLAYER_HEIGHT) > FIRE_Y) && ((player->y+CONFIG_PLAYER_HEIGHT) <= CONFIG_GROUND_Y)) {

        return true;
    } else {
        return false;
    }
}
//Check if there is a collsiions with water
bool obstacles_collision_water(player_t *player) {
        if(player->playerNum) {
        return false;
    }
    if((player->x < WATER_X2) && ((player->x+CONFIG_PLAYER_WIDTH) > WATER_X1) && ((player->y+CONFIG_PLAYER_HEIGHT) > WATER_Y) && ((player->y+CONFIG_PLAYER_HEIGHT) <= CONFIG_GROUND_Y)) {

        return true;
    } else {
        return false;
    }
}
//Check if there is collision with goo
bool obstacles_collision_goo(player_t *player) {
    if((player->x < GOO_X2) && ((player->x+CONFIG_PLAYER_WIDTH) > GOO_X1) && ((player->y+CONFIG_PLAYER_HEIGHT) > GOO_Y) && ((player->y+CONFIG_PLAYER_HEIGHT) <= CONFIG_PLATFORM_Y_1)) {
        return true;
    } else {
        return false;
    }
}

bool obstacles_collision_door(player_t *player) {
    bool doorOpened = door_open();
    if((player->ground_y == CONFIG_PLATFORM_Y_2) && !doorOpened && ((player->x+CONFIG_PLAYER_WIDTH) > DOOR_X) && (player->x < (DOOR_X+DOOR_W))) {
        return true;
    } else {
        return false;
    }
}

bool obstacles_game_over() {
    return game_over_flag;
}

//Check to see if the players are in the spots to win.
bool obstacles_players_win(player_t *player1, player_t *player2) {
    bool check1 = false;
    bool check2 = false;
    //check 1 checks the fireboys position in the final door to win the game
    if(((player1->x+CONFIG_PLAYER_WIDTH)>CONFIG_FINAL_DOOR_X_FIRE) && (player1->x<(CONFIG_FINAL_DOOR_X_FIRE+CONFIG_FINAL_DOOR_W))) {
        check1 = true;
    }
    if(((player2->x+CONFIG_PLAYER_WIDTH)>CONFIG_FINAL_DOOR_X_WATER) && (player2->x<(CONFIG_FINAL_DOOR_X_WATER+CONFIG_FINAL_DOOR_W))) {
        check2 = true;
    }

    return (check1 && check2);

}