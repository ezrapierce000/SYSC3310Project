/* 	Final project for SYSC3310
*  	Author: Ezra Pierce
*		Date: December 11, 2020
*  
*  	Note: This file contains portions on the provided example program.
* 	The copyright is available at the bottom of this file.
*
*/

#include "msp.h"
#include <stdbool.h>

#define FORWARD_SWITCH_PIN (BIT1)
#define BACKWARD_SWITCH_PIN (BIT4)
#define SWITCH_PINS (FORWARD_SWITCH_PIN|BACKWARD_SWITCH_PIN)
#define RED_LED_PIN (BIT0)
#define RGB_LED_PINS (BIT0|BIT1|BIT2)
#define GREEN_LED_PIN (BIT1)
#define UART_PINS (BIT2|BIT3)
	
	
/* 
*		0b00: State 1, no LEDs on
*		0b01: State 2, Red LED on
*		0b10: State 3, Green LED on
*		0b11: State 4, Both LEDs on
*/
volatile uint8_t ledState = 0x00;
	
void turnOffAll(void){
	P1OUT &= ~RED_LED_PIN;
	P2OUT &= ~RGB_LED_PINS;
}


// Function updates LEDs based on state and sends new state to host CPU
void update(){
	turnOffAll();
	switch(ledState){
		case 0:
			EUSCI_A0->TXBUF = '1';
			break;
		case 1:
			P1OUT |= (uint8_t)RED_LED_PIN;
			EUSCI_A0->TXBUF = '2';
			break;
		case 2:
			P2OUT |= (uint8_t)GREEN_LED_PIN;
			EUSCI_A0->TXBUF = '3';
			break;
		case 3:
			P1OUT |= (uint8_t)RED_LED_PIN;
			P2OUT |= (uint8_t)GREEN_LED_PIN;
			EUSCI_A0->TXBUF = '4';
			break;	
		default:
			EUSCI_A0->TXBUF = '0';
			break;
	}
}
	

// TRUE is to advance one state, FALSE is go back one state
void changeState(bool direction){
	if(direction){
		if(ledState == 3 || ledState > 3){
			ledState = 0;
		} else{
			ledState++;
		}
	} else{
		if(ledState == 0 || ledState < 0){
			ledState = 3;
		} else{
			ledState--;
		}	
	}
	update();
}

void ioCfg(){
	// Configuring switches and LEDs as IO
	P1SEL0 &= (uint8_t)(~RED_LED_PIN)|(~SWITCH_PINS);
	P1SEL1 &= (uint8_t)(~RED_LED_PIN)|(~SWITCH_PINS);
	P2SEL0 &= (uint8_t)~RGB_LED_PINS;
	P2SEL1 &= (uint8_t)~RGB_LED_PINS;
	
 	// Configure UART pins to secondary function
	P1SEL0 |= (uint8_t)(UART_PINS);

	// Set as switches as input with pull-up resistor
	P1DIR &= (uint8_t)~SWITCH_PINS;
	P1REN |= (uint8_t)SWITCH_PINS;
	P1OUT |= (uint8_t)SWITCH_PINS;
	
	// Set LEDs as output
	P1DIR |= (uint8_t)RED_LED_PIN;
	P2DIR |= (uint8_t)(RGB_LED_PINS);
}

void interruptCfg(){
	// Enable switch interrupts
	P1IFG = 0x00;
	P1IE |= (uint8_t)(SWITCH_PINS);
	P1IES |= (uint8_t)(SWITCH_PINS);
}

void uartCfg(){
	// The following 33 lines have been taken from the  'MSP432P401 Demo - eUSCI_A0 UART echo at 9600 baud using BRCLK = 12MHz'
	// Copyright is included at the bottom of this file
	CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
	CS->CTL0 = 0;                           // Reset tuning parameters
	CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
	CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
					CS_CTL1_SELS_3 |                // SMCLK = DCO
					CS_CTL1_SELM_3;                 // MCLK = DCO
	CS->KEY = 0;                            // Lock CS module from unintended accesses

	// Configure UART pins
	P1->SEL0 |= UART_PINS;                // set 2-UART pin as secondary function

	// Configure UART
	EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
	EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
					EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
	// Baud Rate calculation
	// 12000000/(16*9600) = 78.125
	// Fractional portion = 0.125
	// User's Guide Table 21-4: UCBRSx = 0x10
	// UCBRFx = int ( (78.125-78)*16) = 2
	EUSCI_A0->BRW = 78;                     // 12000000/16/9600
	EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
					EUSCI_A_MCTLW_OS16;

	EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
	EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
	EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
}

void nvicCfg(){
	// Setting up interrupts
	NVIC_SetPriority(PORT1_IRQn, 2);
  	NVIC_ClearPendingIRQ(PORT1_IRQn);
	NVIC_EnableIRQ(PORT1_IRQn);
	NVIC_SetPriority(EUSCIA0_IRQn, 3);
  	NVIC_ClearPendingIRQ(EUSCIA0_IRQn);
	NVIC_EnableIRQ(EUSCIA0_IRQn);
}

int main(){
	// Turning off watchdog
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; 
	
	switchCfg();
	interruptCfg();

	// Enable sleep on exit from ISR
	SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	
	uartCfg();

	// Enable global interrupt
	__enable_irq();

	nvicCfg();
	
	// Init LEDs
	turnOffAll();
	
	while(1){
		__ASM("WFI");
			}
}

// ISR to handle switch presses
void PORT1_IRQHandler (void){
	if(P1IFG & (uint8_t)FORWARD_SWITCH_PIN){
		P1IFG &= (uint8_t)~FORWARD_SWITCH_PIN;
		changeState(true);
	} else{
		P1IFG &= (uint8_t)~BACKWARD_SWITCH_PIN;
		changeState(false);
	}
}

// UART rcv ISR
void EUSCIA0_IRQHandler(void){		
    // Check if interrupt is from receive buffer
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
				
				// Read message from host CPU
				uint8_t buf = EUSCI_A0->RXBUF;
			 /*
				*	Message codes:
				*	0xFF: return current state
				* 0xFE:	go forward one state
				* 0xFD:	go backward one state
				*/
				if(buf == 0xFF){
					switch(ledState){
						case 0:
							EUSCI_A0->TXBUF = '1';
							break;
						case 1:
							EUSCI_A0->TXBUF = '2';
							break;
						case 2:
							EUSCI_A0->TXBUF = '3';
							break;
						case 3:
							EUSCI_A0->TXBUF = '4';
							break;
						default:
							EUSCI_A0->TXBUF = '1';
					}
				} else if(buf == 0xFE){
						changeState(true);
				} else if(buf == 0xFD){
						changeState(false);
				}
    }
		// Clear interrupt flag
		EUSCI_A0->IFG &= ~(EUSCI_A_IFG_RXIFG);
}




/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP432 CODE EXAMPLE DISCLAIMER
 *
 * MSP432 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see http://www.ti.com/tool/mspdriverlib for an API functional
 * library & https://dev.ti.com/pinmux/ for a GUI approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP432P401 Demo - eUSCI_A0 UART echo at 9600 baud using BRCLK = 12MHz
//
//  Description: This demo echoes back characters received via a PC serial port.
//  SMCLK/ DCO is used as a clock source and the device is put in LPM0
//  The auto-clock enable feature is used by the eUSCI and SMCLK is turned off
//  when the UART is idle and turned on when a receive edge is detected.
//  Note that level shifter hardware is needed to shift between RS232 and MSP
//  voltage levels.
//
//  The example code shows proper initialization of registers
//  and interrupts to receive and transmit data.
//
//                MSP432P401
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//            |                 |
//            |     P1.3/UCA0TXD|----> PC (echo)
//            |     P1.2/UCA0RXD|<---- PC
//            |                 |
//
//   William Goh
//   Texas Instruments Inc.
//   June 2016 (updated) | June 2014 (created)
//   Built with CCSv6.1, IAR, Keil, GCC
//******************************************************************************



		