//XRTIC Gripper Example Code
//filename: main.c
//Authors: Matthew Bocharnikov, Zhiheng Luo, Cody Luong, Amarchand Niranjan, William Ogle
//Last Updated: 5/3/2021
//Description: Demonstrates simple 180 degree servo gripper functionality on the RSLK robot

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <servo180.h>

//Subsystem Master Clock Frequency
#define SYS_CLK         48000000

//Servo struct
Servo180 servoSettings;

//Click Button 2 to toggle open and close the gripper
void PORT1_IRQHandler(void) //BUTTON 2 IRQ
{
    toggleOpenClose(&servoSettings);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
}

//Make sure to follow the instructions in the user
//manual about setting up the gripper
int main(void)
{
    //enable button 2 interrupt
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN4, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    Interrupt_enableInterrupt(INT_PORT1);

    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    //connect gripper signal to P2.4
    //Make sure the open and close degree settings are chosen correctly
    servo180InitArgs(&servoSettings,SYS_CLK,85,30,GPIO_PORT_P2,GPIO_PIN4);
    moveServoToDegree(90, &servoSettings);
}
