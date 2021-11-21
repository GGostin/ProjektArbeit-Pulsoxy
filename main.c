
#include <msp430.h>
#include "PCD8544.h"
#include <stdio.h>

#define LCD5110_SCLK_PIN            BIT5
#define LCD5110_DN_PIN              BIT7
#define LCD5110_SCE_PIN             BIT0
#define LCD5110_DC_PIN              BIT1
#define LCD5110_SELECT              P6OUT &= ~LCD5110_SCE_PIN
#define LCD5110_DESELECT            P6OUT |= LCD5110_SCE_PIN
#define LCD5110_SET_COMMAND         P1OUT &= ~LCD5110_DC_PIN
#define LCD5110_SET_DATA            P1OUT |= LCD5110_DC_PIN
#define LCD5110_COMMAND             0
#define LCD5110_DATA                1

unsigned int i = 0;
unsigned int DataReadyRed = 0;
unsigned int DataReadyInfraRed = 0;

typedef enum{ DCRed = 0, ACRed, DCInfra,ACInfra, Temp, LiPo}ADCTYPE;

unsigned int ADCValue[6];//
unsigned int numInterruptA = 0;
unsigned int numInterruptB = 0;

void InitLCDPins(void);

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    /*  Default settings after reset:
    *
    *   Source of the Main system clock (MCLK) and sub-main system clock (SMCLK) is
    *   Internal Digitally Controlled Oscillator Clock (DCO).
    *   MCLK is about 1 MHz.
    *   VLO is about 32 kHz.
    *
    *
    */

    //Use PIn 5.0 and 5.1 for switching the LEDs

    P5DIR |= BIT0 | BIT1;
    P5SEL0 |= BIT0 | BIT1;




    //Configuration for ADC Pin start on DC for RED (A3, P1.3)
    //Later each ADC Pin (A1, A3, A10, A11) is configured and triggered manually

    ADCCTL0 |= ADCON ;
    ADCCTL2 &= ~ADCRES;
    ADCCTL2 |= ADCRES_2; // 12 Bit Conversion (Takes 14 Clock Cycles)
    ADCMCTL0 |= ADCINCH_3; // Channel A3
    ADCCTL0 |= ADCENC; //Enable Conversion

    InitLCDPins();

     //Setup UCAO




    __delay_cycles(300000); // we have to wait a bit for the LCD to boot up.
    initLCD();

    clearLCD();

    __delay_cycles(500000); //So that we come from a fres boot the Screen will be empty and doesn't accidentally contain old Information

    TB2CCR0  = 781-1;          //Every 1 ms  the LED is already on for 110 us and will continue for another 110 us
    TB2CCTL1 |= OUTMOD_2;    //TB2CCR1 toggle/set
    TB2CCR1  = 86-1;         //0..110us , 1.89ms ... 2ms
    TB2CCTL2 |= OUTMOD_6;    //TBCCR2 reset/set
    TB2CCR2  = 695-1;         //0.890 ms - 1.11 ms
    TB2CTL   |= TBSSEL_2 | MC_3 | TBIE |TBCLR; // SMCLK, Up-Down-Mode, Interrupt Enable on Max Value and Zero
    TB2CCTL0 |= CCIE; //Enable Interrupt when CCR0 is reached.

    PM5CTL0 &= ~LOCKLPM5; //Without this the pins won't be configured in Hardware.
    TB2CTL &= ~TBIFG;
    TB2CCTL0 &= ~CCIFG;

    __bis_SR_register(GIE);


    setAddr(8,0);
    writeStringToLCD("SPO2");

    setAddr(60,0);
    writeStringToLCD("bpm");

    setAddr(10,3);
    writeStringToLCD("94%");
    setAddr(62,3);
    writeStringToLCD("62");

    clearBank(5);
    setAddr(38,5);
    writeBattery(BATTERY_FULL);

    while(1)
    {
    }


} // eof main

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER2_B1_VECTOR
__interrupt void TIMER2_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_B1_VECTOR))) TIMER2_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{

    //#TODO#//
    //DO The ADC Conversion of the InfraRed here.

    switch(TB2IV)
    {
    case TBIV_14: numInterruptA++; break; //TBIFG
    case TBIV_2: numInterruptB++; break; //CCRIFG

    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_B0_VECTOR))) TIMER2_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    //Now this is really interesting. In Up/Down Mode when an Interrupt Is set when CCR0 is reached, the TB2IV Vector doesnt get anything. It just stays empty whiel going in a B0 ISR
    //The TBIFG interrupt when reaching 0 on the other hands goes into TIMER2_B1_ISR and creates an entry in TB2IV.

    //#TODO#//
    //Do the ADC Conversion of the Red LED here.

    numInterruptB++;

}

void InitLCDPins(void)
{

    P1SEL0 |= LCD5110_DN_PIN | LCD5110_SCLK_PIN;
    P1OUT &= ~BIT0;

    P6OUT |= LCD5110_SCE_PIN;
    P6DIR |= LCD5110_SCE_PIN;

    P1OUT |= LCD5110_DC_PIN;  // Disable LCD, set Data mode
    P1DIR |= LCD5110_DC_PIN;  // Set pins to output direction

    UCA0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
    UCA0CTLW0 |= UCMST|UCSYNC|UCCKPH|UCMSB;   // 3-pin, 8-bit SPI master
    UCA0CTLW0 |= UCSSEL_3; // SMCLK
    UCA0BRW   |= 0; //1:1


    UCA0CTLW0 &= ~UCSWRST;

}

void setAddr(unsigned char xAddr, unsigned char yAddr) {
    writeToLCD(LCD5110_COMMAND, PCD8544_SETXADDR | xAddr);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETYADDR | yAddr);
}

void writeToLCD(unsigned char dataCommand, unsigned char data) {
    LCD5110_SELECT;

    if(dataCommand) {
        LCD5110_SET_DATA;
    } else {
        LCD5110_SET_COMMAND;
    }

    UCA0TXBUF  = data;
    while(!(UCTXIFG & UCA0IFG ))

    LCD5110_DESELECT;
}

void initLCD() {
    writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETVOP | 0x3F);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETTEMP | 0x02);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETBIAS | 0x03);
    writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET);
    writeToLCD(LCD5110_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

void writeCharToLCD(char c) {
    unsigned char i;
    for(i = 0; i < 5; i++) {
        writeToLCD(LCD5110_DATA, font[c - 0x20][i]);
    }
    writeToLCD(LCD5110_DATA, 0);
}

void writeStringToLCD(const char *string) {
    while(*string) {
        writeCharToLCD(*string++);
    }
}

void writeBattery(const int i)
{
    unsigned char j;
    for(j = 0 ; j < 7; j++)
    {
        writeToLCD(LCD5110_DATA, Battery[i][j]);
    }
    writeToLCD(LCD5110_DATA, 0);

}

void clearLCD() {
    setAddr(0, 0);
    int i = 0;
    while(i < PCD8544_MAXBYTES) {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, 0);
}

void clearBank(unsigned char bank) {
    setAddr(0, bank);
    int i = 0;
    while(i < PCD8544_HPIXELS) {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, bank);
}

