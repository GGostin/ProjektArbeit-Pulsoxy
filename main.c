
#include <msp430.h>
#include "PCD8544.h"

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
    //Need TB2xxx

    P5DIR |= BIT0 | BIT1;
    P5SEL0 |= BIT0 | BIT1;

    TB2CCR0  = 1000-1;          //Every 1 ms  the LED already on for 110 us and will continue for another 110 us
    TB2CCTL1 = OUTMOD_2;    //TB2CCR1 toggle/set
    TB2CCR1  = 110-1;         //0..110us , 1.89ms ... 2ms
    TB2CCTL2 = OUTMOD_6;    //TBCCR2 reset/set
    TB2CCR2  = 890-1;         //0.890 ms - 1.11 ms
    TB2CTL   = TBSSEL_1 | MC_3;

    //Use Pin 6.3 , 6.4 (TB3.4, TB3.5) as PWM for the LEDs intensity

    TB3CCR0  = 1000 -1; //PWM 1 ms Period
    TB3CCTL4 = OUTMOD_7; //CCR4 reset/set
    TB3CCTL5 = OUTMOD_7; //CCR5 reset/set
    TB3CCR4  = 500-1;
    TB3CCR5  = 700-1;
    TB3CTL   = TBSSEL__SMCLK | MC__UP | TBCLR;



    InitLCDPins();

    // Setup UCAO

    PM5CTL0 &= ~LOCKLPM5;



    __delay_cycles(500000);
    initLCD();

    clearLCD();

    setAddr(8,0);
    writeStringToLCD("SPO2");

    setAddr(60,0);
    writeStringToLCD("bpm");

    setAddr(10,3);
    writeStringToLCD("94%");
    setAddr(62,3);
    writeStringToLCD("62");

    while(1)
    {
        clearBank(5);
        __delay_cycles(100000);
        setAddr(38,5);
        writeBattery(BATTERY_FULL);
        __delay_cycles(2000000);
        setAddr(38,5);
        writeBattery(BATTERY_75);
        __delay_cycles(2000000);
        setAddr(38,5);
        writeBattery(BATTERY_50);
        __delay_cycles(2000000);
        setAddr(38,5);
        writeBattery(BATTERY_25);
        __delay_cycles(2000000);

        for(i = 0; i < 6; i++)
        {
            setAddr(38,5);
            writeBattery(BATTERY_10);
            __delay_cycles(500000);
            setAddr(38,5);
            writeBattery(BATTERY_25);
            __delay_cycles(500000);
        }
    }
;


} // eof main

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

