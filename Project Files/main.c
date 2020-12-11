/*****************************************************************************
*
* Copyright (C) 2013 - 2017 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the
*   distribution.
*
* * Neither the name of Texas Instruments Incorporated nor the names of
*   its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*
* MSP432 empty main.c template
*
******************************************************************************/

#include "msp.h"

#define BUTTON_ADDRESS 0x12
#define STATE0 0
#define STATE1 1
#define STATE2 2
#define STATE3 3
#define DEBOUNCE_TIME 100000

static void fwd(void);
static void bwd(void);

void PORT1_IRQHandler(void);
 //governs our LED logic
volatile uint8_t state;

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; //Disable watchdog timer

    CS->KEY = CS_KEY_VAL;
    CS->CTL0 = 0;
    CS->CTL0 = CS_CTL0_DCORSEL_3;
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
    CS->KEY = 0;

    P1->SEL0 |= BIT2 | BIT3;

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;
    EUSCI_B_CTLW0_SSEL__SMCLK;
    EUSCI_A0->BRW = 78;
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

    __enable_irq();

    //GPIO config to set P1.1 (L) and P1.4 (R) as inputs
    P1->SEL0 &= (uint8_t)(~((1 << 4) | (1 << 1) | (1<<0)));
    P1->SEL1 &= (uint8_t)(~((1 << 4) | (1 << 1) | (1<<0)));
    P1->DIR |= (uint8_t)((0 << 4) | (0 << 1));
    P1->REN |= (uint8_t)((1 << 4) | (1 << 1));
    P1->OUT |= (uint8_t)((1 << 4) | (1 << 1));

    P1->IE |= (uint8_t)(BUTTON_ADDRESS); //Interrupt enable to 10010 -> pin 1 and 4
    P1->IES |= (uint8_t)(BUTTON_ADDRESS); //falling edge event
    P1->IFG &= (uint8_t)~(BUTTON_ADDRESS); // 01101 clear flag1

    NVIC_SetPriority(PORT1_IRQn, 2);
    NVIC_ClearPendingIRQ(PORT1_IRQn);
    NVIC_EnableIRQ(PORT1_IRQn);
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);

    //initialize our 2 LED's
    P1->SEL0 &= (uint8_t)(~((1 << 0)));
    P1->SEL1 &= (uint8_t)(~((1 << 0)));
    P1->DIR |= (uint8_t)((1 << 0));
    P1->OUT &= (uint8_t)(~((1 << 0)));

    P2->SEL0 &= (uint8_t)(~(1 << 2));
    P2->SEL1 &= (uint8_t)(~(1 << 2));
    P2->DIR |= (uint8_t)((1 << 2));
    P2->OUT &= (uint8_t)(~(1<<2));

    state = STATE0;

   for(;;){

       __WFI();

   }

   return 0;

}

void PORT1_IRQHandler(void)
{

    if(P1->IFG & (1<<4)){

        fwd();
        P1->IFG &= ~(1<<4);

    }



    if(P1->IFG & (1<<1)){

        bwd();
        P1->IFG &= ~(1<<1);

    }



}

void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

        // Echo the received character back
        EUSCI_A0->TXBUF = EUSCI_A0->RXBUF;
    }
}

static void fwd(void)
{

    if(state==STATE0){

        P1->OUT &= (uint8_t)(~(1<<0));
        P2->OUT |= (uint8_t)(1<<2);
        state=STATE1;

    }else if(state==STATE1){

        P1->OUT |= (uint8_t)(1<<0);
        P2->OUT &= (uint8_t)(~(1<<2));
        state=STATE2;

    }else if(state==STATE2){

        P1->OUT |= (uint8_t)(1<<0);
        P2->OUT |= (uint8_t)(1<<2);
        state=STATE3;

    }else{

        P1->OUT &= (uint8_t)(~(1<<0));
        P2->OUT &= (uint8_t)(~(1<<2));
        state=STATE0;

    }


}

static void bwd(void)
{

    if(state==STATE0){

        P1->OUT |= (uint8_t)(1<<0);
        P2->OUT |= (uint8_t)(1<<2);
        state=STATE3;

    }else if(state == STATE3){

        P1->OUT |= (uint8_t)(1<<0);
        P2->OUT &= (uint8_t)(~(1<<2));
        state=STATE2;

    }else if(state==STATE2){

        P1->OUT &= (uint8_t)(~(1<<0));
        P2->OUT |= (uint8_t)(1<<2);
        state=STATE1;

    }else if(state==STATE1){

        P1->OUT &= (uint8_t)(~(1<<0));
        P2->OUT &= (uint8_t)(~(1<<2));
        state=STATE0;

    }


}



