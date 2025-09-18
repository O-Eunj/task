#include <stdio.h>
#include "main.h"
#include "task.h"


int counter = 0;
int flag_toggle = 0;
int interval = 0;
uint8_t blink_count = 0;

int button_pressed = 0;
int button_hold_counter = 0;

void task_led(void) {
    if(flag_toggle) {
        if(blink_count > 0) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); 
            if(blink_count != 0xff) {
                blink_count--;
            }
        }
        flag_toggle = 0;
    }
}

void task_timer(void) {
    counter++;
    if(counter >= interval) {
        flag_toggle = 1;
        counter = 0;
    }
    
    push_button();
}

void push_button(void) {
    static int prev_state = 1;
    int state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    if(prev_state == 1 && state == 0) {
        if(interval < 5000) {
            interval += 50;
        }        
        button_pressed =1;
        button_hold_counter = 0;
    } else if(prev_state == 0 && state == 1) {
        button_pressed = 0;
        button_hold_counter = 0;
    }
    
    prev_state = state;

    if(button_pressed) {
      button_hold_counter++;
      if(button_hold_counter >= 200) {
        if(interval < 5000) {
          interval += 50;
        }
        button_hold_counter = 0;
      }
    }
}
