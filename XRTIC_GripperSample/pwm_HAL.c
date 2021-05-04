//XRTIC Gripper Example Code
//filename: pwm_HAL.c
//Authors: Matthew Bocharnikov, Zhiheng Luo, Cody Luong, Amarchand Niranjan, William Ogle
//Last Updated: 5/3/2021
//Description: Demonstrates simple 180 degree servo gripper functionality on the RSLK robot


#include <pwm_HAL.h>

//timer A
//up-mode to set frequency
// - TAxCCR0 cannot be used as output
// - TAxCCR0 is the max count value of TAxR
//Reset/set output mode
///

//Array of valid PWM pins
PWM_Pin PWM_Pins [] = {
                   //TA0
                   {TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, {GPIO_PORT_P2, GPIO_PIN4}},
                   {TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, {GPIO_PORT_P2, GPIO_PIN5}},
                   {TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, {GPIO_PORT_P2, GPIO_PIN6}},
                   {TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, {GPIO_PORT_P2, GPIO_PIN7}},
                   //TA1
                   {TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, {GPIO_PORT_P7, GPIO_PIN7}},
                   {TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, {GPIO_PORT_P7, GPIO_PIN6}},
                   {TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, {GPIO_PORT_P7, GPIO_PIN5}},
                   {TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, {GPIO_PORT_P7, GPIO_PIN4}},
                   //TA2
                   {TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, {GPIO_PORT_P5, GPIO_PIN6}},
                   {TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, {GPIO_PORT_P5, GPIO_PIN7}},
                   {TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, {GPIO_PORT_P6, GPIO_PIN6}},
                   {TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, {GPIO_PORT_P6, GPIO_PIN7}},
                   //TA3
                   {TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, {GPIO_PORT_P10, GPIO_PIN5}},
                   {TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, {GPIO_PORT_P8, GPIO_PIN2}},
                   {TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, {GPIO_PORT_P9, GPIO_PIN2}},
                   {TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, {GPIO_PORT_P9, GPIO_PIN3}}
};

uint8_t prescalars [] = {
                         TIMER_A_CLOCKSOURCE_DIVIDER_1,
                         TIMER_A_CLOCKSOURCE_DIVIDER_2,
                         TIMER_A_CLOCKSOURCE_DIVIDER_3,
                         TIMER_A_CLOCKSOURCE_DIVIDER_4,
                         TIMER_A_CLOCKSOURCE_DIVIDER_5,
                         TIMER_A_CLOCKSOURCE_DIVIDER_6,
                         TIMER_A_CLOCKSOURCE_DIVIDER_7,
                         TIMER_A_CLOCKSOURCE_DIVIDER_8,
                         TIMER_A_CLOCKSOURCE_DIVIDER_10,
                         TIMER_A_CLOCKSOURCE_DIVIDER_12,
                         TIMER_A_CLOCKSOURCE_DIVIDER_14,
                         TIMER_A_CLOCKSOURCE_DIVIDER_16,
                         TIMER_A_CLOCKSOURCE_DIVIDER_20,
                         TIMER_A_CLOCKSOURCE_DIVIDER_24,
                         TIMER_A_CLOCKSOURCE_DIVIDER_28,
                         TIMER_A_CLOCKSOURCE_DIVIDER_32,
                         TIMER_A_CLOCKSOURCE_DIVIDER_40,
                         TIMER_A_CLOCKSOURCE_DIVIDER_48,
                         TIMER_A_CLOCKSOURCE_DIVIDER_56,
                         TIMER_A_CLOCKSOURCE_DIVIDER_64
};

//Calculates the required prescalar based on clock frequency and
//desired output frequency for TimerA_PWM_Config struct
bool calcPrescalar(PWM_Params *pwm_settings)
{
    if(pwm_settings->freq > 0)
    {
        int8_t prescalar = pwm_settings->sys_clk / (pwm_settings->freq * 65536);
        uint8_t scalar_index;

        for(scalar_index = 0;scalar_index < NUM_PRESCALARS;scalar_index++)
        {
            if(prescalars[scalar_index] > prescalar)
            {
                prescalar = prescalars[scalar_index];
                pwm_settings->prescalar = prescalar;
                return true;
            }
        }
    }

    return false;
}

//Calculates the required CCR0 value based on clock frequency,
//desired output frequency, and prescalar for TimerA_PWM_Config struct
//The CCR0 value is the value that the Timer A counter register counts to
//and which sets the frequency
bool calcCCR0Value(PWM_Params *pwm_settings)
{
    if(pwm_settings->freq > 0)
    {
        pwm_settings->ccr0_value = (uint16_t)(pwm_settings->sys_clk / (pwm_settings->freq * pwm_settings->prescalar));
        return true;
    }
    else
        return false;
}

//Correlates TA base number and CCR of desired output pin
bool findPinIndex(PWM_Params *pwm_settings)
{
    int8_t pin_index;

    for(pin_index = 0;pin_index < NUM_PWM_PINS;pin_index++)
    {
        if(PWM_Pins[pin_index].pin.portNum == pwm_settings->output_pin.portNum &&
                PWM_Pins[pin_index].pin.pinNum == pwm_settings->output_pin.pinNum)
        {
            pwm_settings->pin_index = pin_index;
            return true;
        }
    }
    return false;
}

//A function that configures PWM by passing arguments
void setPWMArgs(PWM_Params *pwm_settings, uint32_t _sys_clk, double _freq, double _dutyCycle, uint16_t _portNum, uint16_t _pinNum)
{
    pwm_settings->sys_clk = _sys_clk;
    pwm_settings->freq = _freq;
    pwm_settings->dutyCycle = _dutyCycle;
    pwm_settings->output_pin.portNum = _portNum;
    pwm_settings->output_pin.pinNum = _pinNum;

    setPWM(pwm_settings);
}

//Sets up PWM by accepting a fully configured pwm struct
void setPWM(PWM_Params *pwm_settings)
{
    //Calculate PWM values:
    //calcPrescalar must be called first
    if(!calcPrescalar(pwm_settings)) while(1){}    //invalid frequency

    //calcCCR0Value must be called second
    if(!calcCCR0Value(pwm_settings)) while(1){}   //invalid frequency

    //findPinIndex can be called either 3rd or 4th
    if(!findPinIndex(pwm_settings)) while(1){}    //invalid output pin

    //calcDutyValue can be called either 3rd or 4th
    if(!calcDutyValue(pwm_settings)) while(1){}   //invalid duty cycle

    //TimerA config
    pwm_settings->PWM_config.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    pwm_settings->PWM_config.clockSourceDivider = pwm_settings->prescalar;
    pwm_settings->PWM_config.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    pwm_settings->PWM_config.compareRegister = PWM_Pins[pwm_settings->pin_index].ccrx;
    pwm_settings->PWM_config.dutyCycle = pwm_settings->duty_value;
    pwm_settings->PWM_config.timerPeriod = pwm_settings->ccr0_value;
    //Generate PWM function sets in Up mode

    //for debugging
    //uint32_t A_freq = CS_getACLK();
    //uint32_t M_freq = CS_getMCLK();

    GPIOInit(pwm_settings);

}

//Calculates the value of the CCR value which will produce the desired duty cycle
bool calcDutyValue(PWM_Params *pwm_settings)
{
    if(pwm_settings->dutyCycle >= 0 && pwm_settings->dutyCycle <= 1)  //ensure duty cycle is within bounds
    {
        pwm_settings->duty_value = (uint16_t)(pwm_settings->dutyCycle * pwm_settings->ccr0_value);
        return true;
    }
    else
        return false;
}

//A function that can be called at run-time to change the pwm frequency
bool updateFrequency(double freq, PWM_Params *pwm_settings)
{
    if(freq > 0)
    {
        pwm_settings->freq = freq;
        calcPrescalar(pwm_settings);
        calcCCR0Value(pwm_settings);
        pwm_settings->PWM_config.clockSourceDivider = pwm_settings->prescalar;
        pwm_settings->PWM_config.timerPeriod = pwm_settings->ccr0_value;
        return true;
    }
    else
        return false;
}

//A function that can be called at run-time to change the pwm duty cycle
bool updateDutyCycle(double duty_cycle, PWM_Params *pwm_settings)
{
    if(duty_cycle >= 0 && duty_cycle <= 1)
    {
        pwm_settings->dutyCycle = duty_cycle;
        calcDutyValue(pwm_settings);
        pwm_settings->PWM_config.dutyCycle = pwm_settings->duty_value;
        return true;
    }
    else
        return false;
}

//Must be called every time the PWM settings are updated
//Generates the PWM signal as configured in the PWM_config struct
void generatePWM(PWM_Params *pwm_settings)
{
    //Generates a PWM with timer running in up mode
    Timer_A_generatePWM(PWM_Pins[pwm_settings->pin_index].timer, &pwm_settings->PWM_config);
}

//Initializes the correct GPIO pins for outputting PWM
void GPIOInit(PWM_Params *pwm_settings)
{
    GPIO_setAsOutputPin(pwm_settings->output_pin.portNum, pwm_settings->output_pin.pinNum);
    GPIO_setOutputLowOnPin(pwm_settings->output_pin.portNum, pwm_settings->output_pin.pinNum);
    GPIO_setAsPeripheralModuleFunctionOutputPin(pwm_settings->output_pin.portNum, pwm_settings->output_pin.pinNum, GPIO_PRIMARY_MODULE_FUNCTION);

}
