#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include"pins.h"




int8_t initHw()
{
    // CLOCK CONFIGURATION
    // Following procedure from p231 datasheet to transition to 16MHz clock
    // Extra commands added to ensure value in every field
    SYSCTL_RCC_R &= ~SYSCTL_RCC_MOSCDIS;                                // Enable main oscillator
    SYSCTL_RCC_R |= SYSCTL_RCC_BYPASS;                                  // Bypass PLL to change frequency
    SYSCTL_RCC_R &= ~SYSCTL_RCC_USESYSDIV;                              // Disable SYSDIV
    SYSCTL_RCC_R &= ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);         // Clear XTAL and OSCSRC fields
    SYSCTL_RCC_R |= SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN;     // Set XTAL to 16MHz and OSCSRC to main
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWRDN;                                  // Clear PWRDN
    SYSCTL_RCC_R &= ~SYSCTL_RCC_SYSDIV_M;                               // Clear SYSDIV
    SYSCTL_RCC_R |=  SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S); // Use divisor of 5 (SYSDIV=4) p223  ==> 40MHz
    while (!(SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS));                       // Wait for PLL to lock
    SYSCTL_RCC_R &= ~SYSCTL_RCC_BYPASS;                                 // Use PLL clock
    SYSCTL_RCC_R &= ~SYSCTL_RCC_ACG;                                    // Disable ACG (irrelevant to this project)
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                               // Clear PWMDIV
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;         // Set up PWM clock


    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;


    // CLOCK GATING
    // Enable GPIO ports ABCDEF peripherals
    // Enable Timers
    SYSCTL_RCGCGPIO_R = BT_0 | BT_1 | BT_2 | BT_3 | BT_4 | BT_5;


    // CONFIGURE BUTTONS
    // Pin Mapping:  PD6 PD7 PF4

    // Define inputs (clear bits)
    GPIO_PORTD_DIR_R &= ~(  BT_6 | BT_7  )  ;
    GPIO_PORTF_DIR_R &= ~(  BT_4  )  ;

    // Enable pins for digital (set bits)
    GPIO_PORTD_DEN_R |=  (  BT_6 | BT_7  ) ;
    GPIO_PORTF_DEN_R |=  (  BT_4 )  ;

    // Enable pull up resistors (set bits)
    GPIO_PORTD_PUR_R |=  (  BT_6 | BT_7  ) ;
    GPIO_PORTF_PUR_R |=  (  BT_4  )  ;



    // CONFIGURE LED
    // SMALL    RED:PC4  BLUE:PF2
    // STRIP    BLUE:PC5
    // RGB      RGBRED:PD0 RGBGREEN: PD1 RGBBLUE:PE4
    // Configure pins as outputs with 2mA strength

    GPIO_PORTC_DIR_R  |=BT_4 | BT_5 ;
    GPIO_PORTC_DR2R_R |=BT_4 | BT_5 ;
    GPIO_PORTC_DEN_R  |=BT_4 | BT_5 ;

    GPIO_PORTD_DIR_R  |=BT_1 ;
    GPIO_PORTD_DR2R_R |=BT_1 ;
    GPIO_PORTD_DEN_R  |=BT_1 ;

    GPIO_PORTE_DIR_R  |=BT_4;
    GPIO_PORTE_DR2R_R |=BT_4;
    GPIO_PORTE_DEN_R  |=BT_4;


    GPIO_PORTF_DIR_R  |= BT_2 ;
    GPIO_PORTF_DR2R_R |= BT_2 ;
    GPIO_PORTF_DEN_R  |= BT_2 ;




/*

    // CONFIGURE Ethernet
    // based on ENC28J60 chip
    // CS: PB0  Clk: PB4  MISO PB6  MOSI PB7

    // Enable pins for digital (set bits)
    GPIO_PORTB_DEN_R |=  (    BT_7 | BT_6 | BT_5 | BT_4   )  ;

    // Define inputs (clear bits)
    GPIO_PORTB_DIR_R &= ~(           BT_6                 )  ;

    // Set pull-up resistors
//    GPIO_PORTB_PUR_R |=  (           BT_6                 )  ;
    GPIO_PORTB_PDR_R |=  (                         BT_4   )  ;

    // Define outputs (set bits)
    GPIO_PORTB_DIR_R |=  (    BT_7 |        BT_5 | BT_4   )  ;

    // Set output level
    GPIO_PORTB_DR2R_R |=  (   BT_7 |        BT_5 | BT_4   )  ;

//    *BITBAND(GPIO_PORTB_DATA_R,5) = 0;
*/
/*
    // Set alternative functions
    GPIO_PORTB_AFSEL_R |= (   BT_7 | BT_6 | BT_5 | BT_4  )  ;
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB7_SSI2TX;

    // Enable clock gating and wait a little
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");

    // Disable SSI2 and set other fields while here EOT=0   MS=0   SSE=0   LBM=0
    SSI2_CR1_R &= ~( SSI_CR1_SSE);
    SSI2_CR1_R &= ~(SSI_CR1_EOT | SSI_CR1_MS | SSI_CR1_LBM);

    // Select PLL clock
    SSI2_CC_R = SSI_CC_CS_SYSPLL;

    // Set the prescale to 2 (minimum value).  Leave SCR field of CR0 at zero.  20MHz result.  p969 Datasheet
    SSI2_CPSR_R = 2;

    // SCR=0   SPH=0   SPO=0   FRF=0   DSS=7 (8bit data) Top 16 bits are reserved and untouched
    SSI2_CR0_R &= 0xffff0000;
    SSI2_CR0_R |= 0x0007;
    SSI2_CR0_R |= 0x2 << 3;


    SSI2_CR1_R |= SSI_CR1_SSE;
*/
/*
    // CONFIGURE TIMER0 (Flashes lights at opening)
    SYSCTL_PPTIMER_R |=               BT_0             ;
    SYSCTL_RCGCTIMER_R |=             BT_0             ;
    TIMER0_CTL_R &= ~(TIMER_CTL_TAEN);
    TIMER0_CFG_R = 0;
    TIMER0_TAMR_R = 2;
    TIMER0_TAILR_R = 16666667;
    TIMER0_IMR_R |=                   BT_0             ;
    TIMER0_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R |= (1 << 19);
    // CONFIGURE TIMER1 (Time limit on escape sequence)
    SYSCTL_PPTIMER_R |=         BT_1                   ;
    SYSCTL_RCGCTIMER_R |=       BT_1                   ;
    TIMER1_CTL_R &= ~(TIMER_CTL_TAEN);
    TIMER1_CFG_R = 0;
    TIMER1_TAMR_R = 1;
    TIMER1_TAILR_R = 95000;                                 // set timer interval to about 1.43ms
    TIMER1_IMR_R |=  TIMER_IMR_TATOIM;
    NVIC_EN0_R |= (1 << 21);                                // Enable interrupts for A
*/



/*


    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB7_M0PWM1;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;               // leave reset state
    while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
    PWM0_0_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;               // turn-off PWM0 generator 0
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
                                                     // Turn on when number hit going down; turn off at load
    PWM0_0_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                     // also using 4096 because load overrides cmpb
    PWM0_0_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
    PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;                // turn-on PWM0 generator 0
    PWM0_ENABLE_R = PWM_ENABLE_PWM1EN;               // enable outputs


*/






    return 0;
}




int main(void)
{
    initHw();
    while (1)
    {
        LED_SMRED      = 1;
        LED_STRIPBLUE  = 1;
        LED_SMBLUE     = 1;
        LED_RGBRED     = 1;
        LED_RGBBLUE    = 1;
        LED_RGBGREEN   = 1;

        int i;
        for (i=0; i<10000000; i++);

        LED_SMRED      = 0;
        LED_STRIPBLUE  = 0;
        LED_SMBLUE     = 0;
        LED_RGBRED     = 0;
        LED_RGBBLUE    = 0;
        LED_RGBGREEN   = 0;


    }

	return 0;
}
