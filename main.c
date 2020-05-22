#include "main.h"
#include "hal_LCD.h"

#define cliBufferSize 15
static _Bool uartReceived = false;                          /* UART receive flag */
char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
static uint8_t cliBuffer[cliBufferSize];                    /* CLI output buffer */
static uint8_t cliIndex = 0;                                /* CLI buffer index */

/******************************************************************/
/*                           GLOBALS                              *
 * ****************************************************************/
/* HELPERS */
int reed_fired_zone;
char zones[] = {'1', '2', '3', '4'};
char zone_states[5]; //Tracks breached/unbreached zones
char nums[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

/* STATES */
enum states{INIT, ARMED, UNARMED, BREACHED, MONITORING};

/* TIMER */
int countdown;
int timer_value;
_Bool one_sec_passed = false;
_Bool done = false;


void main(void)
{
    initialize_board();

    enum states alarm_state = INIT;

    reset_zone_states_lcd();


    while (1)
    {
        switch (alarm_state)
        {
            case INIT:
            {
                done = false;
                reset_alarm();
                init_led_unarmed();
                displayScrollText("SET A TIMER");
                clearLCD();
                set_time();

                countdown = timer_value;
                alarm_state = UNARMED;

                break;
            }
            case ARMED:
            {
                init_led_armed();
                alarm_state = MONITORING;
                break;
            }
            case UNARMED:
            {
                start_countback();
                if(done){
                    displayStaticText("ARMED");
                    __delay_cycles(1000000);
                    alarm_state = ARMED;
                }

                break;
            }
            case BREACHED:
            {
                displayStaticText("BREACHED");
                zone_states[reed_fired_zone-1] = 'B';
                showChar(' ', pos5);
                showChar(' ', pos6);

                // Continue listening even after breach
                alarm_state = MONITORING;

                break;

            }
            case MONITORING:
            {
                display_zone_states_lcd();
                display_ok_zones_led(zone_states);

                reed_fired_zone = reed_fire();
                if (reed_fired_zone > 0)
                {
                    sound_alarm();
                    alarm_state = BREACHED;
                    clearLCD();
                    break;
                }
                if (GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0){
                   alarm_state = INIT;
                }
//                break;
            }
        }
    }

}


void display_zone_states_lcd(){
    showChar(zone_states[0], pos1);
    showChar(zone_states[1], pos2);
    showChar(zone_states[2], pos3);
    showChar(zone_states[3], pos4);
    showChar(zone_states[4], pos5);

}

const int NUM_ZONES = 4;

void toggle_green_led_on(int zone_index)
{
    switch (zone_index)
    {
    case 0:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN1); //ZONE 1 GREEN
        break;
    case 1:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0); //ZONE 2 GREEN
        break;
    case 2:
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5); //ZONE 3 GREEN
        break;
    case 3:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3); //ZONE 4 GREEN
        break;
    }
}

void toggle_green_led_off(int zone_index)
{
    switch (zone_index)
    {
    case 0:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN1); //ZONE 1 GREEN
        break;
    case 1:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); //ZONE 1 GREEN
        break;
    case 2:
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5); //ZONE 1 GREEN
        break;
    case 3:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3); //ZONE 1 GREEN
        break;
    }
}

const int FLASH_DELAY = 100;
void orange_and_red_led_on(int zone_index)
{

    int i;
    switch (zone_index)
    {
        case 0:
        {
            for (i = 0; i < 5; i++)
            {
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); //ZONE 1 RED
                __delay_cycles(FLASH_DELAY);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //ZONE 1 ORANGE
                __delay_cycles(FLASH_DELAY);
            }
            break;

        }

        case 1:
        {
            for (i = 0; i < 5; i++)
            {
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1); //ZONE 2 RED
                __delay_cycles(FLASH_DELAY);
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1); //ZONE 2 ORANGE
                __delay_cycles(FLASH_DELAY);

            }
            break;

        }

        case 2:
        {
            for (i = 0; i < 5; i++)
            {
                GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2); //ZONE 3 RED
                __delay_cycles(FLASH_DELAY);
                GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2); //ZONE 3 ORANGE
                __delay_cycles(FLASH_DELAY);

            }
            break;


        }

        case 3:
        {
            for (i = 0; i < 5; i++)
            {
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); //ZONE 4 RED
                __delay_cycles(FLASH_DELAY);
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); //ZONE 4 ORANGE
                __delay_cycles(FLASH_DELAY);

            }
            break;
        }
    }
}

void display_ok_zones_led(char* zone_states){

    /* Continuously toggles the green LEDs to indicate whether zone is ok or not */

    int zone;
    for(zone = 0; zone < NUM_ZONES; zone++){

        if(zone_states[zone] == 'G'){
            toggle_green_led_on(zone);
        }
        else{
            toggle_green_led_off(zone);
            orange_and_red_led_on(zone);
        }
    }
}

void init_led_unarmed(){
    /*Setting low on these pins turns the LEDs on that are on the inverted signal (red)*/

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //ZONE 1 RED
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1); //ZONE 2 RED
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2); //ZONE 3 RED
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); //ZONE 4 RED
}

void init_led_armed()
{
    /*Setting high on these pins turns the LEDs on that are on the not on the inverted signal (orange)*/

    /*Yellow LEDs represent the armed state*/
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); //ZONE 1 ORANGE
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1); //ZONE 2 ORANGE
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2); //ZONE 3 ORANGE
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); //ZONE 4 ORANGE

}

void reset_zone_states_lcd(){
    memset(zone_states, 0, sizeof(zone_states));
    int i;
    for(i = 0; i < 4; i++){
        zone_states[i] = 'G';
    }
    zone_states[4] = ' '; // To prevent a garbage value
}

int reed_fire(){
    /* Returns which zone the reed switch went off in */

    if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2) == 0)        return 1;
    else if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == 0)    return 2;
    else if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 0)    return 3;
    else if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0)    return 4;
    else return 0;
}


void start_countback( void )
{
    char countdown_char[10];

    //triggers the interrupt
    if (one_sec_passed) /* each second */
    {
        one_sec_passed = false;
        if (countdown >= 1)
        {
            sprintf(countdown_char, "%d", countdown);

            if(countdown >= 10){
                showChar(countdown_char[0], pos3);
                showChar(countdown_char[1], pos4);
            }
            else{
                showChar('0', pos3);
                showChar(countdown_char[0], pos4);
            }
            countdown--;
        }
        else
        {
            done = true;
            countdown = 12;
        }

    }

}


void show_time_lcd(int timer_val_l){
    showChar(nums[timer_val_l], pos3);
    showChar('0', pos4);
    __delay_cycles(100000);
}


void set_time()
{
    timer_value = 0;

    int timer_val_l = 1; /* Start timer at 10 seconds by default */
    static int timer_val_r = 0; /* Will always be 0 */

    int time_decr_btn;
    int time_incr_btn;
    int both_pressed = 0;

    clearLCD();
    showChar('D', pos2);
    showChar('U', pos5);

    while (!both_pressed)
    {
        both_pressed = time_incr_btn && time_decr_btn;
        time_decr_btn = GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0;
        time_incr_btn = GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0;


        if (both_pressed){
            timer_value = timer_val_l*10;
            break;
        }
        else
        {
            if ((time_decr_btn) && (timer_val_l >= 1))
            {
                timer_val_l -= 1;
                show_time_lcd(timer_val_l);
            }
            if ((time_incr_btn) && (timer_val_l <= 8))
            {
                timer_val_l++;
                show_time_lcd(timer_val_l);
            }
        }

        show_time_lcd(timer_val_l);
    }
}



void initialize_board()
{
    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
//    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
//    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

   /**************************************************************************
    * REED SWITCHES INITIALIZATION                                           *
    **************************************************************************/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN2); /* ZONE 1*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN3); /* ZONE 2*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3); /* ZONE 3*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4); /* ZONE 4*/

   /**************************************************************************
    * MICROPHONE INITIALIZATION                                              *
    **************************************************************************/
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0);

   /**************************************************************************
    * SPEAKER INITIALIZATION                                                 *
    **************************************************************************/
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);

   /**************************************************************************
    * CONTROL BUTTONS INITIALIZATION                                         *
    **************************************************************************/
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0); /* Arms the system */

    /* RTC */
    RTC_init(RTC_BASE, 32768, RTC_CLOCKPREDIVIDER_1);
    RTC_clearInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT_FLAG);
    RTC_enableInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT);
    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);

}



void sound_alarm()
{
    Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
}


void reset_alarm(void)
{
    //turn off buzzer
    Timer_A_stop(TIMER_A0_BASE);

    //set 8 pins on low
    //zone 1
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN1);   //s2
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);   //s1
    //zone 2
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);   //s2
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);   //s1
    //zone 3
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);   //s2
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);   //s1
    //zone 4
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);   //s2
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);   //s1

    clearLCD();
    reset_zone_states_lcd();

}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);


    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}


void uartDisplay(uint8_t *sendText, uint8_t length)
{
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */

    int i;
    for (i = 0 ; i < length ; i++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, sendText[i]); /* send message */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    }

    if ((sendText[0] != '>') && (sendText[0] != '#') && (sendText[0] != ' ')) /* if not enter key or welcome message, it was command, make new line */
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '>'); /* send new prompt */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 32); /* send space*/
    }
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN1 + GPIO_PIN2,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}


/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN,
                                                GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
    ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE,
                                               ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

// RTC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR (void)
{
    switch (__even_in_range(RTCIV,2)){
        case 0: break;  //No interrupts
        case 2:         //RTC overflow
            one_sec_passed = true;
            break;
        default: break;
    }
}


