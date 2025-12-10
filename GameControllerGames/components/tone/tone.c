#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "tone.h"

#define TWICE_LOW_FREQ LOWEST_FREQ*2
#define NUM_TONES 5
#define BIAS 0x80
#define AMPLITUDE 0xFF


//Coefficients
#define PI2 2*M_PI
#define HALVER 2
#define DOUBLE 2
#define QUARTER 0.25f
#define THREE_QUARTERS 0.75f



uint8_t *myBuf;
uint32_t sampleRate = 0;





int32_t tone_init(uint32_t sample_hz) {
    //Check the frequency bounds and initialize buffer
    if(sample_hz > TWICE_LOW_FREQ) {
    sound_init(sample_hz);
    myBuf = (uint8_t*) malloc(sizeof(uint8_t) * sample_hz/LOWEST_FREQ);
    sampleRate = sample_hz;
    }
    return 0;
}


int32_t tone_deinit(void) {
    //DEinitialize sound and free buffer
    sound_deinit();
    free(myBuf);
    return 0;
}


void tone_start(tone_t tone, uint32_t freq) {
    uint32_t numSamples = 0;
    //bound check for tone and freq
    if(freq < TWICE_LOW_FREQ || tone > NUM_TONES) {
        return;
    }
    numSamples = sampleRate/freq;
    float_t slopeSaw = (float)AMPLITUDE/(numSamples);
    float_t slopeTri = ((float)AMPLITUDE*DOUBLE)/(numSamples);

    //Cases for different tones
    switch(tone) {
        case SINE_T:
            //Assign each value of i a sine value
            for(uint32_t i = 0; i < numSamples; i++) {
                myBuf[i] = (BIAS-1)*sinf(((float)i/numSamples)*PI2) + BIAS;
            }
        break;
        case SQUARE_T: 
            // Set values to max for half the period and 0 for other half
            for(uint32_t i = 0; i < numSamples; i++) {
                if(i < numSamples/HALVER) {
                    myBuf[i] = AMPLITUDE;
                }
                else {
                    myBuf[i] = 0;
                }
                
            }
        break;
        case TRIANGLE_T:
            /*set values for 1st and last quarter of period to upward sloped line
            and middle of period 1/4 - 3/4 to downward sloped line*/
            for(uint32_t i = 0; i < numSamples; i++) {
                if (i <= (numSamples*QUARTER)) {
                    myBuf[i] = slopeTri*i + BIAS;
                }
                else if (i < (numSamples*THREE_QUARTERS)){
                    myBuf[i] = -slopeTri*i + BIAS;
                }
                else {
                    myBuf[i] = slopeTri*i + BIAS + DOUBLE;
                }
            }
        break;
        case SAW_T:
            //start the upward line over every half period
            for(uint32_t i = 0; i < numSamples; i++) {
                if (i < (numSamples)/2) {
                    myBuf[i] = ((float)i * slopeSaw) + BIAS;
                }
                else {
                    myBuf[i] = ((float)i * slopeSaw) - BIAS;
                }

            }        
        break;
        case LAST_T:
            for(uint32_t i = 0; i < numSamples; i++) {
                myBuf[i] = 1;
            }
        break;
        default: {
            for(uint32_t i = 0; i < numSamples; i++) {
                myBuf[i] = 1;
            }
        }
        
    }
    sound_cyclic(myBuf, numSamples);





}