#include "switches.h"
#include "config.h"
#include <stdio.h>



#define O CONFIG_COLOR_BACKGROUND //CONFIG_COLOR_BACKGROUND //Background COlor
#define R rgb565(254, 140, 0) //Organge 1
#define A rgb565(241, 91, 6) //Orange 2
#define G rgb565(230, 175, 6) //Yellow Gold
#define P rgb565(58, 1, 95) //Dark purple
#define N rgb565(99, 1, 151) //purple 2
#define I rgb565(185, 1, 150) //Pink
#define U rgb565(163, 1, 150)//pink2
#define Y YELLOW
#define L BLACK

#define LEVER_OFFSET (LEVER_W/4) //hitbox width for the character to trigger the lever.
#define MOVE_LEFT 1
#define MOVE_RIGHT 2

trigger_t lever;
trigger_t button1;
trigger_t button2;


bool flipped = false;


//Initialize the trigger to off state
void switches_init() {
    lever.on = false;
    lever.type = true;

    button1.on = false;
    button1.type = false;

    button2.on = false;
    button2.type = false;


}

//Switch the trigger to on state
void switches_activate_trigger(trigger_t *trigger) {
    trigger->on = true;
}
//Deactivate switch
void switches_deactivate_trigger(trigger_t *trigger) {
    trigger->on = false;
}

//Returns the state of the button 1 
bool switches_get_button1() {
    return button1.on;

}

//Returns the state of the button 2 
bool switches_get_button2() {
    return button2.on;
}

//Returns the state of the lever 
bool switches_get_lever() {
    return lever.on;
}

void trigger_draw() {
    //Draw the lever
    if(lever.on) {
        lcd_drawRGBBitmap(CONFIG_LEVER_X, CONFIG_LEVER_Y, lever_on, LEVER_W, LEVER_H);

    } else {
        lcd_drawRGBBitmap(CONFIG_LEVER_X, CONFIG_LEVER_Y, lever_off, LEVER_W, LEVER_H);
    }
    //draw 1st button
    if(button1.on) {
        lcd_drawRGBBitmap(CONFIG_BUTTON1_X, CONFIG_BUTTON1_Y, button_on, BUTTON_W, BUTTON_H);
    } else {
        lcd_drawRGBBitmap(CONFIG_BUTTON1_X, CONFIG_BUTTON1_Y, button_off, BUTTON_W, BUTTON_H);
    }
    //Draw second button
    if(button2.on) {
        lcd_drawRGBBitmap(CONFIG_BUTTON2_X, CONFIG_BUTTON2_Y, button_on, BUTTON_W, BUTTON_H);
    } else {
        lcd_drawRGBBitmap(CONFIG_BUTTON2_X, CONFIG_BUTTON2_Y, button_off, BUTTON_W, BUTTON_H);
    }
    

}

//Tick fnxn for telling if the player is hitting the lever or button
void switches_tick(player_t *player1, player_t *player2) {
    static bool leverHitLeft = false;
    static bool leverHitRight = false;


    //checking the conditions for being on the button  for both players
    if(((player1->x+CONFIG_PLAYER_WIDTH)>CONFIG_BUTTON1_X) && 
        (player1->x<(CONFIG_BUTTON1_X+BUTTON_W)) && 
        (player1->ground_y == CONFIG_PLATFORM_Y_2)) {

        player1->onBtn1 = true;
    } else {
        player1->onBtn1 = false;
    }
    if(((player2->x+CONFIG_PLAYER_WIDTH)>CONFIG_BUTTON1_X) && 
        (player2->x<(CONFIG_BUTTON1_X+BUTTON_W)) && 
        (player2->ground_y == CONFIG_PLATFORM_Y_2)) {

        player2->onBtn1 = true;
    } else {
        player2->onBtn1 = false;
    }

    //checking the conditions for being on the button 2  for both players
    if(((player1->x+CONFIG_PLAYER_WIDTH)>CONFIG_BUTTON2_X) && 
        (player1->x<(CONFIG_BUTTON2_X+BUTTON_W)) && 
        (player1->ground_y == CONFIG_PLATFORM_Y_2)) {

        player1->onBtn2 = true;
    } else {
        player1->onBtn2 = false;
    }
    if(((player2->x+CONFIG_PLAYER_WIDTH)>CONFIG_BUTTON2_X) && 
        (player2->x<(CONFIG_BUTTON2_X+BUTTON_W)) && 
        (player2->ground_y == CONFIG_PLATFORM_Y_2)) {

        player2->onBtn2 = true;
    } else {
        player2->onBtn2 = false;
    }


    //Check to see if either player is standing right next to the lever on both sides to trigger it.
    if((player1->ground_y == CONFIG_PLATFORM_Y_1) && 
        (player1->moveState == MOVE_LEFT) && 
        (player1->x >= (CONFIG_LEVER_X+(LEVER_W-LEVER_OFFSET))) && 
        (player1->x <= (CONFIG_LEVER_X+LEVER_W))) {

        leverHitRight = true;
    } else if ((player2->ground_y == CONFIG_PLATFORM_Y_1) && 
            (player2->moveState == MOVE_LEFT) && 
            (player2->x >= (CONFIG_LEVER_X+(LEVER_W-LEVER_OFFSET))) && 
            (player2->x <= (CONFIG_LEVER_X+LEVER_W))) {

        leverHitRight = true;
    } else {
        leverHitRight = false;
    }
    //make sure the player is running the direction to actually trigger it.
    if((player1->ground_y == CONFIG_PLATFORM_Y_1) && 
        (player1->moveState == MOVE_RIGHT) &&  
        ((player1->x+CONFIG_PLAYER_WIDTH) >= CONFIG_LEVER_X) && 
        ((player1->x+CONFIG_PLAYER_WIDTH) <= (CONFIG_LEVER_X+LEVER_W))) {

        leverHitLeft = true;
    } else if ((player2->ground_y == CONFIG_PLATFORM_Y_1) && 
            (player2->moveState == MOVE_RIGHT) && 
            ((player2->x+CONFIG_PLAYER_WIDTH) >= CONFIG_LEVER_X) && 
            ((player2->x+CONFIG_PLAYER_WIDTH) <= (CONFIG_LEVER_X+LEVER_W))) {

        leverHitLeft = true;
    } else {
        leverHitLeft = false;
    }

    //CHECK to change the lever position
    if(!lever.on) {
        //check flag that tracks the lever being hit.
        if(leverHitRight && !flipped) {
            lever.on = true;
            flipped = true;
        } else if (!leverHitRight && flipped) {
            flipped = false;
        }
        
    } else {
        //check flag that tracks the lever being hit.
        if(leverHitLeft && !flipped) {
            lever.on = false;
            flipped = true;
        } else if (!leverHitLeft && flipped) {
            flipped = false;
        }

    }


    //Turning on the button
    if(player1->onBtn1 || player2->onBtn1) {
        button1.on = true;
    } else {
        button1.on = false;
    }

    //Turning on the button 2
    if(player1->onBtn2 || player2->onBtn2) {
        button2.on = true;
    } else {
        button2.on = false;
    }

}

const color_t lever_on[] = {
    O, O, O, O, Y, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, Y, G, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, Y, G, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O,
    R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R,
    A, A, R, R, R, R, R, A, A, R, R, R, R, R, A, A, A, A, R, R, R, R, R, A, A, R, R, R, R, R, A, A,
    R, R, A, R, R, R, A, R, R, A, R, R, R, A, R, R, R, R, A, R, R, R, A, R, R, A, R, R, R, A, R, R,
    R, R, R, A, R, A, R, R, R, R, A, R, A, R, R, R, R, R, R, A, R, A, R, R, R, R, A, R, A, R, R, R,
    R, R, R, R, A, R, R, R, R, R, R, A, R, R, R, R, R, R, R, R, A, R, R, R, R, R, R, A, R, R, R, R,

};

const color_t lever_off[] = {
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, Y, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, G, Y, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, G, Y, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, Y, G, Y, Y, G, Y, Y, G, Y, O, O, O, O, O, O, O, O, O, O, O, O,
    R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R,
    A, A, R, R, R, R, R, A, A, R, R, R, R, R, A, A, A, A, R, R, R, R, R, A, A, R, R, R, R, R, A, A,
    R, R, A, R, R, R, A, R, R, A, R, R, R, A, R, R, R, R, A, R, R, R, A, R, R, A, R, R, R, A, R, R,
    R, R, R, A, R, A, R, R, R, R, A, R, A, R, R, R, R, R, R, A, R, A, R, R, R, R, A, R, A, R, R, R,
    R, R, R, R, A, R, R, R, R, R, R, A, R, R, R, R, R, R, R, R, A, R, R, R, R, R, R, A, R, R, R, R,

};

const color_t button_on[] = {
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, O, O, O, O, O, O, O,
    O, O, O, O, O, O, I, I, U, U, U, U, U, U, U, U, U, U, U, U, U, U, U, U, I, I, O, O, O, O, O, O,
    O, O, O, O, O, I, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, I, O, O, O, O, O,
    O, O, O, O, O, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, O, O, O, O, O,
    O, O, O, O, O, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, O, O, O, O, O,
    P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P,
    N, N, P, P, P, P, P, N, N, P, P, P, P, P, N, N, N, N, P, P, P, P, P, N, N, P, P, P, P, P, N, N,
    P, P, N, P, P, P, N, P, P, N, P, P, P, N, P, P, P, P, N, P, P, P, N, P, P, N, P, P, P, N, P, P,
    P, P, P, N, P, N, P, P, P, P, N, P, N, P, P, P, P, P, P, N, P, N, P, P, P, P, N, P, N, P, P, P,
    P, P, P, P, N, P, P, P, P, P, P, N, P, P, P, P, P, P, P, P, N, P, P, P, P, P, P, N, P, P, P, P,
};

const color_t button_off[] = {
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, O, O, O, O, O, O, O,
    O, O, O, O, O, O, I, I, U, U, U, U, U, U, U, U, U, U, U, U, U, U, U, U, I, I, O, O, O, O, O, O,
    O, O, O, O, O, I, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, I, O, O, O, O, O,
    O, O, O, O, O, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, O, O, O, O, O,
    O, O, O, O, O, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, O, O, O, O, O,
    O, O, O, O, O, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, O, O, O, O, O,
    O, O, O, O, O, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, O, O, O, O, O,
    O, O, O, O, I, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, I, O, O, O, O,
    O, O, O, I, I, U, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, U, I, I, O, O, O,
    P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P, P,
    N, N, P, P, P, P, P, N, N, P, P, P, P, P, N, N, N, N, P, P, P, P, P, N, N, P, P, P, P, P, N, N,
    P, P, N, P, P, P, N, P, P, N, P, P, P, N, P, P, P, P, N, P, P, P, N, P, P, N, P, P, P, N, P, P,
    P, P, P, N, P, N, P, P, P, P, N, P, N, P, P, P, P, P, P, N, P, N, P, P, P, P, N, P, N, P, P, P,
    P, P, P, P, N, P, P, P, P, P, P, N, P, P, P, P, P, P, P, P, N, P, P, P, P, P, P, N, P, P, P, P,


};