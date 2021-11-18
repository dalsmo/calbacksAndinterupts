#ifndef TIMER_H
#define TIMER_H

#include <stddef.h>

typedef void (*functionToCall_t)(void);
static functionToCall_t TC;

int register_Callback(functionToCall_t TC_temp, int delay_in_10ms);
int initialise_Timer(void);
int execute_Callback(void);


#endif // TIMER_H