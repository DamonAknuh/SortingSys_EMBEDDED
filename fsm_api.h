/*
 * fsm.h
 *
 * Created: 2020-11-20 11:27:54 AM
 *  Author: hunka
 */ 


#ifndef FSM_H_
#define FSM_H_

/**********************************************************************
** ___  ____ ____ _ _  _ ____ ____
** |  \ |___ |___ | |\ | |___ [__
** |__/ |___ |    | | \| |___ ___]
**
***********************************************************************/

typedef enum
{
    INIT_STATE          = 0b00000001,
    IDLE_STATE          = 0b00000010,
    NEW_OBJ_STATE       = 0b00000100,

    CLASS_STATE         = 0b10000000,
    POS_TRAY_HARD       = 0b00100000,
} systemStates_e;


extern volatile uint8_t  g_CurrentState;

#define TRIGGER_STATE(STATE) (g_CurrentState |= STATE)
#define PROCESS_STATE(STATE) (g_CurrentState &= ~STATE)

/**********************************************************************
** ____ _  _ _  _ ____ ___ _ ____ _  _ ____
** |___ |  | |\ | |     |  | |  | |\ | [__
** |    |__| | \| |___  |  | |__| | \| ___]
**
***********************************************************************/

void IdleState();
void InitState();
void PositionTrayState();
void NewObjState();
void ClassifyState();



#endif /* FSM_H_ */