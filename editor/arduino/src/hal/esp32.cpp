#include <stdlib.h>
extern "C" {
 #include "openplc.h"
}
#include "Arduino.h"
#include "../examples/Baremetal/defines.h"

#include "ESP32_FastPWM.h"

//OpenPLC HAL for ESP32 boards

/******************PINOUT CONFIGURATION**************************
Digital In:  17, 18, 19, 21, 22, 23, 27, 32 (%IX0.0 - %IX0.7)
             33                             (%IX1.0 - %IX1.0)
Digital Out: 01, 02, 03, 04, 05, 12, 13, 14 (%QX0.0 - %QX0.7)
             15, 16                         (%QX1.0 - %QX1.1)
Analog In:   34, 35, 36, 39                 (%IW0 - %IW2)
Analog Out:  25, 26                         (%QW0 - %QW1)
*****************************************************************/

//Create the I/O pin masks
uint8_t pinMask_DIN[] = {PINMASK_DIN};
uint8_t pinMask_AIN[] = {PINMASK_AIN};
uint8_t pinMask_DOUT[] = {PINMASK_DOUT};
uint8_t pinMask_AOUT[] = {PINMASK_AOUT};

    #define NUM_OF_PWM_PINS       8 //only 8 pins possible with independent frequency

    #define PWM_CHANNEL_0_PIN     4
    #define PWM_CHANNEL_1_PIN     13
    #define PWM_CHANNEL_2_PIN     18
    #define PWM_CHANNEL_3_PIN     19
    #define PWM_CHANNEL_4_PIN     21
    #define PWM_CHANNEL_5_PIN     22
    #define PWM_CHANNEL_6_PIN     32
    #define PWM_CHANNEL_7_PIN     33

    #define TIMER_CHANNEL_0       0
    #define TIMER_CHANNEL_1       2
    #define TIMER_CHANNEL_2       4
    #define TIMER_CHANNEL_3       6
    #define TIMER_CHANNEL_4       8
    #define TIMER_CHANNEL_5       10
    #define TIMER_CHANNEL_6       12
    #define TIMER_CHANNEL_7       14

    #define PWM_RESOLUTION        10 //10-bit
 // Max resolution is 20-bit 
 // Resolution 65536 (16-bit) for lower frequencies, OK @ 1K 
 // Resolution  4096 (12-bit) for lower frequencies, OK @ 10K 
 // Resolution  1024 (10-bit) for higher frequencies, OK @ 50K 
 // Resolution  256  ( 8-bit)for higher frequencies, OK @ 100K, 200K 
 // Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K 



    ESP32_FAST_PWM *PWM_Instance[NUM_OF_PWM_PINS];

    const uint8_t pins[] = {PWM_CHANNEL_0_PIN, PWM_CHANNEL_1_PIN, PWM_CHANNEL_2_PIN, PWM_CHANNEL_3_PIN,
                            PWM_CHANNEL_4_PIN, PWM_CHANNEL_5_PIN, PWM_CHANNEL_6_PIN, PWM_CHANNEL_7_PIN};

    const uint8_t timers[] = {TIMER_CHANNEL_0, TIMER_CHANNEL_1, TIMER_CHANNEL_2, TIMER_CHANNEL_3,
                              TIMER_CHANNEL_4, TIMER_CHANNEL_5, TIMER_CHANNEL_6, TIMER_CHANNEL_7};

extern "C" uint8_t set_hardware_pwm(uint8_t, float, float); //this call is required for the C-based PWM block on the Editor

void hardwareInit()
{
    for (int i = 0; i < NUM_DISCRETE_INPUT; i++)
    {
        pinMode(pinMask_DIN[i], INPUT);
    }
    
    for (int i = 0; i < NUM_ANALOG_INPUT; i++)
    {
        pinMode(pinMask_AIN[i], INPUT);
    }
    
    for (int i = 0; i < NUM_DISCRETE_OUTPUT; i++)
    {
        pinMode(pinMask_DOUT[i], OUTPUT);
    }

    for (int i = 0; i < NUM_ANALOG_OUTPUT; i++)
    {
        pinMode(pinMask_AOUT[i], OUTPUT);
    }
}

//PWM function
uint8_t set_hardware_pwm(uint8_t ch, float freq, float duty)
{
    if (ch >= NUM_OF_PWM_PINS)
    {
        return 0; // Invalid channel number
    }

	// get the pwm pin
    uint8_t pwm_pin = pins[ch];

    // Initialize the PWM instance if not already done
    if (PWM_Instance[ch] == nullptr)
    {
        PWM_Instance[ch] = new ESP32_FAST_PWM(pwm_pin, freq, duty, timers[ch], PWM_RESOLUTION);
        // Disables the pin for other uses as before
	}
	// Update the PWM instance if already exists
    else if (PWM_Instance[ch]->setPWM(pwm_pin, freq, duty))
    {
        return 1;
    }

    return 0;
}


void updateInputBuffers()
{
    for (int i = 0; i < NUM_DISCRETE_INPUT; i++)
    {
        if (bool_input[i/8][i%8] != nullptr) 
            *bool_input[i/8][i%8] = digitalRead(pinMask_DIN[i]);
    }
    
    for (int i = 0; i < NUM_ANALOG_INPUT; i++)
    {
        if (int_input[i] != nullptr)
            *int_input[i] = (analogRead(pinMask_AIN[i]) * 16);
    }
}

void updateOutputBuffers()
{
    for (int i = 0; i < NUM_DISCRETE_OUTPUT; i++)
    {
        if (bool_output[i/8][i%8] != nullptr) 
            digitalWrite(pinMask_DOUT[i], *bool_output[i/8][i%8]);
    }
    for (int i = 0; i < NUM_ANALOG_OUTPUT; i++)
    {
        if (int_output[i] != nullptr) 
            dacWrite(pinMask_AOUT[i], (*int_output[i] / 256));
    }
}
