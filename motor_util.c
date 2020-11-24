/*
 * motor_util.c
 *
 * Created: 2020-11-20 8:17:15 PM
 *  Author: hunka
 */ 
#include <stdlib.h> // the header of the general-purpose standard library of C programming language
#include <avr/io.h>// the header of I/O port
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <stdint.h>
#include <stdbool.h>

#include "project.h"
#include "linkedlist_api.h"

static uint8_t  s_CurrentMotorStep;

// ==> g_HomingFlag = True when tray is homed.
//                  else = false. 
volatile uint8_t  g_HomingFlag; 


#if ENABLE_SMALL_STEPPER

// Small Stepper step table
const uint8_t motorStepTable_g[] = 
{
    0b110000, 
    0b101000,
    0b000110, 
    0b000101
};

 #define MOTOR_STEPS_REV        (2048)
 #define MOTOR_QUARTER_STEPS    (MOTOR_STEPS_REV / 4)
 #define MOTOR_RAMP_STEPS       (MOTOR_QUARTER_STEPS/3)
 #define MOTOR_CONS_STEPS       (MOTOR_RAMP_STEPS)
 #define MOTOR_RAMP_CONS        (MOTOR_RAMP_STEPS + MOTOR_CONS_STEPS)  

#elif !ENABLE_SMALL_STEPPER 

// Large Stepper step table
const uint8_t motorStepTableFWD_g[] =
{
    0b110000,
    0b000110,
    0b101000,
    0b000101
};

 #define MOTOR_STEPS_REV        (200)
 #define MOTOR_QUARTER_STEPS    (MOTOR_STEPS_REV / 4)
 #define MOTOR_RAMP_STEPS       (15)
 #define MOTOR_CONS_STEPS       (20)
 #define MOTOR_RAMP_CONS        (MOTOR_RAMP_STEPS + MOTOR_CONS_STEPS)  


#endif // ENABLE_LARGE_STEPPER

#define SIZEOF_MOTOR_TABLE    (sizeof(motorStepTable_g)/sizeof(motorStepTable_g[0])) 

void mStepMotor(bool dirCW, uint8_t quadrants)
{
    uint32_t i;
    uint32_t steps = quadrants * MOTOR_QUARTER_STEPS;

    if (dirCW)
    {
        // == > Increment Current Step to next step. 
        s_CurrentMotorStep++;

        for (i = 0; i < steps; i++)
        {
            // the steps in the step table (1,2,3,4,1,2...) to turn motor CW
            PORTA = motorStepTable_g[(s_CurrentMotorStep + i) % SIZEOF_MOTOR_TABLE];

            // == > Trapezoidal Acceleration Profiling. 
            if (i < MOTOR_RAMP_STEPS * quadrants)
            {
                mTim1_DelayMs(MOTOR_START_DELAY_MS - (i / quadrants));
            }
            else if (i < MOTOR_CONS_STEPS * quadrants)
            {
                mTim1_DelayMs(MOTOR_END_DELAY_MS);
            }
            else
            {
                mTim1_DelayMs(( i / quadrants) - MOTOR_RAMP_CONS  + MOTOR_END_DELAY_MS);
            }
        }

        s_CurrentMotorStep = (s_CurrentMotorStep + i - 1) % SIZEOF_MOTOR_TABLE;

    }

    else if (!dirCW)
    {
        // == > Decrement Current Step to next step. 
        s_CurrentMotorStep = SIZEOF_MOTOR_TABLE - s_CurrentMotorStep;

        for (i = 0; i < steps; i++)
        {
            // Reverse the steps in the step table (4,3,2,1,4,3...) to turn motor CCW
            PORTA = motorStepTable_g[3 -  ((s_CurrentMotorStep + i ) % SIZEOF_MOTOR_TABLE)];

            // == > Trapezoidal Acceleration Profiling. 
            if (i < MOTOR_RAMP_STEPS * quadrants)
            {
                mTim1_DelayMs(MOTOR_START_DELAY_MS - (i / quadrants));
            }
            else if (i < MOTOR_CONS_STEPS * quadrants)
            {
                mTim1_DelayMs(MOTOR_END_DELAY_MS);
            }
            else
            {
                mTim1_DelayMs(( i / quadrants) - MOTOR_RAMP_CONS  + MOTOR_END_DELAY_MS);
            }
        }

        s_CurrentMotorStep = 3 - ((s_CurrentMotorStep + i - 1) % SIZEOF_MOTOR_TABLE);

    }

}


void mTray_Init(void)
{
    uint32_t i;

    for (i = 0; !g_HomingFlag; i++)
    {
        // the steps in the step table (1,2,3,4,1,2...) to turn motor CW
        PORTA = motorStepTable_g[(s_CurrentMotorStep + i) % SIZEOF_MOTOR_TABLE];
        
        mTim1_DelayMs(MOTOR_START_DELAY_MS);
    }

    s_CurrentMotorStep = (i % 4);

}
