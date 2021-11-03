#include "tm4c123gh6pm.h"

/***********Variables***********/
    int count = 0;
    int current_state = 0;
    int i = 0;
/*******************************/


int main(void)
{
    /***********LEDs***********/

        /*Enable for PORTF Clock gate*/
        SYSCTL_RCGCGPIO_R |= 1 << 5;
        /*Set the Led Direction pin to output*/
        GPIO_PORTF_DIR_R |= (1 << 1) | (1 << 2) | (1 << 3);
        /*Set Led Pin mode to GPIO*/
        GPIO_PORTF_AFSEL_R &= ~((1 << 1) | (1 << 2) | (1 << 3));
        /*Set maximum output current for the LED pin to 2 mA*/
        GPIO_PORTF_DR2R_R |= 1 << 2;
        /*Enable to the Pull down resistor of the LED pin*/
        GPIO_PORTF_PDR_R |= (1 << 1) | (1 << 2) | (1 << 3);
        /*Enable the Digital functionality of the LED pin*/
        GPIO_PORTF_DEN_R |= (1 << 1) | (1 << 2) | (1 << 3);

    /*************************/

    /***********Switch***********/
        /*Unlock PortF0*/
        GPIO_PORTF_LOCK_R = 0x4C4F434B;
        GPIO_PORTF_CR_R |= 1 << 0;
        /*Set the switches direction to input*/
        GPIO_PORTF_DIR_R &= ~((1 << 0) | (1 << 4));
        /*set switches mode to GPIO*/
        GPIO_PORTF_AFSEL_R &= ~((1 << 0) | (1 << 4));
        /*Enable pull up resistor for the Switches pin*/
        GPIO_PORTF_PUR_R |= (1 << 0) | (1 << 4);
        /*Enable the digital functionality of Switches*/
        GPIO_PORTF_DEN_R |= (1 << 0) | (1 << 4);

    /*************************/

    /***********UART***********/

        /*Enable for PORT A Clock gate*/
        SYSCTL_RCGCGPIO_R |= 1 << 0;
        /*Enable the alternate function for port A0 and port A1*/
        GPIO_PORTA_AFSEL_R |= (1 << 0) | (1 << 1);
        /*Enable UART channel 0 clock gate*/
        SYSCTL_RCGCUART_R |= 1<<0;
        /*Disable the open drain connection of PA0 &  PA1*/
        GPIO_PORTA_ODR_R &= ~((1 << 0) | (1 << 1));
        /*Enable the Pull down resistor of PA0 &  PA1*/
        GPIO_PORTA_PDR_R |= (1 << 0) | (1 << 1);
        /*Set the digital enable of PA0 & PA1 */
        GPIO_PORTA_DEN_R |= (1 << 0) | (1 << 1);
        /*Set the selected alternate function*/
        GPIO_PORTA_PCTL_R |= 0x11;
        /*Set the baud rate divisor value for MCU clock = 16000000 & BaudRate = 9600*/
        UART0_IBRD_R =104;
        UART0_FBRD_R = 11;
        /*Set serial parameters*/
        UART0_LCRH_R = 0x60;
        /*Enable UART Module*/
        UART0_CTL_R  |= 1<<0;

    /*************************/

    /***********Timers*********/
        /*Timer A enable*/
        //TIMER0_CTL_R |= 0 << 0;
        /*Enable timer clock gate*/
        SYSCTL_RCGCTIMER_R |= 1 << 0;
        /*GPTM Config*/
        TIMER0_CFG_R = 0x04;
        /*Timer mode*/
        TIMER0_TAMR_R |= 0X02;
        TIMER0_TAMR_R |= 3 << 1;
        TIMER0_TAMR_R |= 2 << 0;
        /*Timer output state*/
        TIMER0_CTL_R |= 6 << 0;
        /*Timer Event mode*/
        TIMER0_CTL_R |= 2 << 0x01;
        TIMER0_CTL_R |= 3 << 0x01;
        //TIMER_CTL_TAEVENT_NEG = 0x01;
        /*Set maximum count in the load interval*/
        TIMER0_TAILR_R = 64000;
        /*Set presacaler value*/
        TIMER0_TAPR_R =250;
        /*Enable and start timer 0 A*/
        TIMER0_CTL_R |= 1 << 0;
        /*Timer match register*/
        TIMER0_TAMATCHR_R |= 1 << 0;

    /*************************/

    /***********Interrupts*********/

        /*Enable timeout interrupt mask*/
        TIMER0_IMR_R |= 1 << 0;
        /*Enable timer interrupt at the NVIC*/
        NVIC_EN0_R |= 1 << 19;

    /*****************************/

        while(1)
        {

        }

        return 0;
}

void Timer0_ISR(void)
{

    char x1[] = {'1','9','8','7','6','5','4','3','2','1'};
    char x21[] = {'3','2','2','2','2','2','2','2','2','2','2','1','1','1','1','1','1','1','1','1','1','9','8','7','6','5','4','3','2','1'};
    char x22[] = {'0','9','8','7','6','5','4','3','2','1','0','9','8','7','6','5','4','3','2','1','0'};

    static unsigned char state = 0x00;
    /*Clear interrupt flag*/
    TIMER0_ICR_R |= 1 << 0;


    if(current_state == 0){
        /******Car Passing State******/
        /******Red Light******/
        /*Turn one Red LED*/
         state ^= 0xff;
         GPIO_PORTF_DATA_BITS_R(1 << 1) = state;
         /*Turn off Green LED*/
         GPIO_PORTF_DATA_BITS_R(1 << 3) = 0x00;
         /*Check if Sw1 is pressed*/
         if(GPIO_PORTF_DATA_BITS_R(1 << 0) == 0x00)
             {
                  current_state =1;
             }
    }
    else if(current_state == 1){
        /******Pedestrians Waiting State******/
        /******Yellow Light******/
        count++;
        /*Turn on Red $ Yellow LEDs*/
         state ^= 0xff;
         GPIO_PORTF_DATA_BITS_R(1 << 1) = state;
         GPIO_PORTF_DATA_BITS_R(1 << 3) = state;

         /*UART Counter for the pedestrians Waiting State*/
             if((UART0_FR_R & (1<<7)) != 0)
             {

                 if(x1[i] != ' ') {
                     if(i == 0) {
                         UART0_DR_R =x1[i];
                         UART0_DR_R = '0';
                         i++;
                     }
                     else{
                         UART0_DR_R =x1[i];
                         i++;
                     }
                 }
             }



         if(count == 12)
         {
             current_state = 2;
             count = 0;
             i = 0;
         }
    }

    else {
        /******Pedestrians Passing State******/
        /******Green Light******/

             count++;
             /*Turn on Green LED*/
             state ^= 0xff;
             GPIO_PORTF_DATA_BITS_R(1 << 3) = state;
             /*Turn off Red LED*/
             GPIO_PORTF_DATA_BITS_R(1 << 1) = 0x00;
             /*UART Counter for the pedestrians Passing State*/
             if((UART0_FR_R & (1<<7)) != 0)
             {
                 if(x21[i] != ' ') {
                     if(i <= 20) {
                         UART0_DR_R = x21[i];
                         UART0_DR_R = x22[i];
                         i++;
                     }
                     else{
                         UART0_DR_R =x21[i];
                         i++;
                     }
                 }
             }
             if(count == 32)
             {
              current_state = 0;
              count = 0;
              i=0;
             }
        }

}

