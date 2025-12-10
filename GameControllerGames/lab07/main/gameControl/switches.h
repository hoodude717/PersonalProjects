#ifndef SWITCHES_H
#define SWITCHES_H

#include <stdint.h>
#include "lcd.h"
#include "player.h"


#define LEVER_W 32
#define LEVER_H 20
#define BUTTON_W 32
#define BUTTON_H 20
#define BASE_H 5
#define NUM_PIXELS 640


extern const color_t lever_on[NUM_PIXELS];
extern const color_t lever_off[NUM_PIXELS];
extern const color_t button_on[NUM_PIXELS];
extern const color_t button_off[NUM_PIXELS];

typedef struct trigger_t
{
    bool type; //true for lever false for button
    bool on;
} trigger_t;

//Initialize the trigger to off state
void switches_init();

//Switch the trigger to on state
void switches_activate_trigger();


//Turn off the trigger
void switches_deactivate_trigger(trigger_t *trigger);

//Tick fnxn for telling if the player is hitting the lever or button
void switches_tick(player_t *player1, player_t *player2);

//Draw the button and lever on the screen
void trigger_draw();

//Returns the state of the button 1 
bool switches_get_button1();

//Returns the state of the button 2 
bool switches_get_button2();

//Returns the state of the lever true is on false is off
bool switches_get_lever();

#endif //SWITCHES_H