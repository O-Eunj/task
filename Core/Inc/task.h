#ifndef TASK_H
#define TASK_H

#include <stdint.h>

extern int interval;
extern uint8_t blink_count;

void task_led(void);
void task_timer(void);
void push_button_press(void);
void push_button_release(void);

#endif