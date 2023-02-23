/*
 * SoftwarePWMExample.c
 *
 *  Created on: Feb 18, 2023
 *      Author: Hunter Geitz
 *
 *      This example controls the LED connected to Pin 1.0 by PWM. You can change the DutyCycle Global variable to change the brightness of the LED. You should vary this to see how the brightness can change.
 *      You can also change this in the Variables/Expressions tab in the debugger to experiment with it as well.
 */

#include <msp430.h>

unsigned short DutyCycle = 500;


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                    // Stop WDT

    // Configure GPIO
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
    
    P6DIR |= BIT6;
    P6OUT &= ~BIT6;

    // Configure P2.3 to an Input
    P2DIR &= ~BIT3;
    P2REN |= BIT3;                  // Enable resistor on P2.3
    P2OUT |= BIT3;
    P2IES |= BIT3; // P2.3 High -> Low edge
    P2IE |= BIT3; // P2.3 interrupt enable
    P2IFG &= ~BIT3; // P2.3 IFG cleared

    // Configure P4.1 to an Input
    P4DIR &= ~BIT1;
    P4REN |= BIT1;                  // Enable resistor on P4.1
    P4OUT |= BIT1;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure Timer_A
    TB0CTL = TBSSEL_2 | MC_1 | TBCLR | TBIE;      // SMCLK, up mode, clear TBR, enable interrupt

    TB0CCTL1 |= CCIE;                             // Enable TB0 CCR1 Interrupt

    TB0CCR1 = DutyCycle;                          // Set CCR1 to the value to set the duty cycle
    TB0CCR0 = 999;                                // Set CCRO to double CCR1
    __bis_SR_register(LPM3_bits | GIE);           // Enter LPM3, enable interrupts
    __no_operation();                             // For debugger
}

// Timer0_B3 Interrupt Vector (TBIV) handler
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(TB0IV,TB0IV_TBIFG))
    {
        case TB0IV_NONE:
            break;                               // No interrupt
        case TB0IV_TBCCR1:
            P1OUT &= ~BIT0;
            break;                               // CCR1 Set the pin to a 0
        case TB0IV_TBCCR2:
            break;                               // CCR2 not used
        case TB0IV_TBIFG:
            P1OUT |= BIT0;                       // overflow Set the pin to a 1
            break;
        default:
            break;
    }
}
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~BIT3;                         // Clear P1.3 IFG
    if (TB0CCR1 >= 999)
        TB0CCR1 = 0;
    else
    TB0CCR1 += 100;
}
