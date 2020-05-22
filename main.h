#ifndef MAIN_H_
#define MAIN_H_

#include "driverlib/driverlib.h"
#include <stdbool.h>
#include <stdio.h>

#define TIMER_A_PERIOD  1000 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      500  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//Output pin to buzzer
#define PWM_PORT        GPIO_PORT_P1
#define PWM_PIN         GPIO_PIN7
//LaunchPad LED1 - note unavailable if UART is used
#define LED1_PORT       GPIO_PORT_P1
#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
#define LED2_PORT       GPIO_PORT_P4
#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC - in this case input A9 maps to pin P8.1
#define ADC_IN_PORT     GPIO_PORT_P8
#define ADC_IN_PIN      GPIO_PIN1
#define ADC_IN_CHANNEL  ADC_INPUT_A9

/* Came with starter code */
void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_PWM(void);
void Init_ADC(void);


/* Declared by us */
/*
 * Pin mapping:
 *
 * -> LEDs
 *
 *    ZONE 1 : GREEN: [P8.1], RED & ORANGE(inv): [P2.7]
 *    ZONE 2 : GREEN: [P8.0], RED & ORANGE(inv): [P5.1]
 *    ZONE 3 : GREEN: [P2.5], RED & ORANGE(inv): [P8.2]
 *    ZONE 4 : GREEN: [P8.3], RED & ORANGE(inv): [P1.5]
 *
 * -> MIC
 *
 * -> SPEAKER
 *
 */


//Init
void initialize_board();

//Timer
void start_count( void );
void start_countback( void );
void set_time();

//LCD
void display_zone_states_lcd();
void reset_zone_states_lcd();
void show_time_lcd(int timer_val_l);

//LEDs
void init_led_unarmed();
void init_led_armed(); //Turns all 4 zone LEDs to orange
void display_ok_zones_led(char* zone_states); // Turns on green LEDs on zones that haven't been breached yet
void toggle_green_led_on(int zone_index);
void toggle_green_led_off(int zone_index);
void orange_and_red_led_on(int zone_index);

//REED
int reed_fire();


Timer_A_outputPWMParam param; //Timer configuration data structure for PWM

#endif /* MAIN_H_ */
