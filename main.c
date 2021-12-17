#include <msp430.h>
#include "PCD8544.h"
#include <math.h>
#include <stdio.h>

//LDC PIN DEFINES
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

//Defines for normal sequence
#define UPDATELCD 5;
#define FULLBATT 4
#define HALFBATT 3
#define MINBATT  2
#define CRITBATT 1


typedef enum{ DCRED = 0, ACRED, DCINFRA, ACINFRA, DCOFF, ACOFF, TEMP, LIPO}ADCTYPE;
unsigned int ADCValue[8];//

unsigned int StartStepA   = 0;
unsigned int StartStepB   = 0;

unsigned int MaxIntensity = 10;

static unsigned int SamplesBPM = 250; //Per 1 sec we take roughly 250 samples of the red LED
static unsigned int NumOfMaxTaken = 4;
static unsigned int SPO2Taken = 2; //Right Shift


static unsigned int HTresholdRed = 0xFFFF;
static unsigned int LTresholdRed = 0;
static unsigned int HTresholdInfra = 0xFFFF;
static unsigned int LTresholdInfra = 0;




typedef enum TagDirection
{
    Rising = 0,
    Falling
}Direction;

typedef struct TagMinMax
{
    unsigned int Max;
    unsigned int Min;
    unsigned int LastValue[5];
    unsigned int NumOfPeaks;
    unsigned int I;
    unsigned int Dir;
}MinMax;

typedef struct TagLEDValues
{
    unsigned long MeanDC;
    unsigned long RmsAC;
}LEDValues;


//Set All Pins to Configure the LDC
void InitLCDPins(void);

//First Takes the AC Part, afterwards the DC Value.
//i is used for the three Cases, Red, InfraRed, Off
void GetDiodeADC(unsigned int chann);

//Take the ADC Value of the NTC and converts it to the equivalent Temperature.
//Used for slight Correction of LEDS;
void GetTemp(unsigned int* meanTemp);

//Take the ADV Value of the LiPo Battery.
//Checks for it to be between 3.7 to 3.0 ??
//Sets Flags to Change the Battery Sign on the LCD
void GetBatteryVoltage(unsigned int* lcdBattFlag);


//Checks the Threshold for either red or infrared.
//Tries to keep it in a range by changing the PWM that controls the Intensity
void CheckLEDIntensity(void);

void CalculateSP02(unsigned int* meanSPO2, unsigned long rms_AC_RED, unsigned long rms_AC_Infra);
int CaluclateBPM(int samples);
void LCDWriteStatusValues(unsigned int SP02, unsigned int bpm, unsigned int battStatus);


void main(void) {

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    MinMax maxDetect;
    memset( &maxDetect, 0, sizeof(maxDetect));
    maxDetect.Dir = Falling;

    unsigned int sampleDC = 0;
    unsigned int sampleAC = 0;

    unsigned int i = 0;
    unsigned int samplesHBeat = 0;
    unsigned int takeBattery  = 0;
    unsigned int updLCD    = 0;
    unsigned int bpm =0;
    unsigned int calcBPM = 0;
    unsigned int firstIterationOver = 0;
    unsigned int maxCheck = 0;
    unsigned int minCheck = 0;
    unsigned int dataValid = 0;

    unsigned int maxACRed   = 0;
    unsigned int minACRed   = 0;

    LEDValues red;
    red.MeanDC = 0;
    red.RmsAC = 0;
    LEDValues infra;
    infra.MeanDC = 0;
    infra.RmsAC = 0;

    unsigned int meanTemp = 0;
    unsigned int voltageLevel = 0;

    unsigned int LCDBattFlag = 0;

    unsigned int SPO2Level = 0;
    unsigned int meanSPO2 = 0;

    //Configuration for ADC Pin start on DC for RED (A3, P1.3)
    //Later each ADC Pin (A1, A3, A10, A11) is configured and triggered manually

    P1SEL0 |= BIT1 | BIT3;
    P1SEL1 |= BIT1 | BIT3;
    P5SEL0 |= BIT2 | BIT3;
    P5SEL1 |= BIT2 | BIT3;

    ADCCTL0  &= ~ADCENC;
    ADCCTL0  |= ADCON;    //ADC-Core on
    ADCCTL1  |= ADCSHP;   //This just makes it so when you start the conversion it stops eventually.
    ADCCTL2  &= ~ADCRES;  //Reset Bit Conversion
    ADCCTL2  |= ADCRES_2; //12 Bit Conversion
    ADCMCTL0 |= ADCINCH_3; // Channel A3
    //ADCIE |= ADCIE0;

    InitLCDPins();
    __delay_cycles(300000); // we have to wait a bit for the LCD to boot up.
    initLCD();
    clearLCD();

    __delay_cycles(500000); //So that we come from a fres boot the Screen will be empty and doesn't accidentally contain old Information

    //Use PIn 5.0 and 5.1 for switching the LEDs
    P5DIR |= BIT0 | BIT1;
    P5SEL0 |= BIT0 | BIT1;

    TB2CCR0  = 2000-1;          //Every 1 ms  the LED is already on for 110 us and will continue for another 110 us
    TB2CCTL1 |= OUTMOD_2;    //TB2CCR1 toggle/set
    TB2CCR1  = 500-1;         //0..110us , 1.89ms ... 2ms
    TB2CCTL2 |= OUTMOD_6;    //TBCCR2 reset/set
    TB2CCR2  = 1500-1;         //0.890 ms - 1.11 ms
    TB2CTL   |= TBSSEL_2 | MC_3 |TBCLR; // SMCLK, Up-Down-Mode, Interrupt Enable on Max Value and Zero
    TB2CTL   |= TBIE;
    TB2CCTL0 |= CCIE; //Enable Interrupt when CCR0 is reached.

    //Two PWM for Intensity
    //P6.3 and P6.4

    P6DIR  |= BIT3 | BIT4;
    P6SEL0 |= BIT3 | BIT4;

    TB3CCR0 = MaxIntensity;
    TB3CCTL4 |= OUTMOD_7;
    TB3CCTL5 |= OUTMOD_7;
    TB3CCR4   = MaxIntensity>>1;
    TB3CCR5   = MaxIntensity>>1;
    TB3CTL   |= TBSSEL__SMCLK  | MC__UP | TBCLR; //Up-Mode 1 MHz

    PM5CTL0 &= ~LOCKLPM5; //Without this the pins won't be configured in Hardware.
    TB2CTL &= ~TBIFG;
    TB2CCTL0 &= ~CCIFG;

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

    __bis_SR_register(GIE);

    while(1)
    {
        if(StartStepA)
        {
            GetDiodeADC(DCRED);

            //Take RMS of AC and Mean of DC
            sampleDC = ADCValue[DCRED];
            sampleAC = ADCValue[ACRED];

            red.RmsAC += (long)(sampleAC * sampleAC) >> 8; //Div by 8 makes it easier
            red.MeanDC += (long)sampleDC >> 8;

            GetTemp(&meanTemp);

            if(!firstIterationOver && dataValid)
            {

                samplesHBeat++;
                MinMaxDetection(&maxDetect, sampleAC);

                if(maxDetect.NumOfPeaks == 5)
                {
                    bpm = CaluclateBPM(samplesHBeat);
                    samplesHBeat = 0;
                    maxDetect.NumOfPeaks = 0;
                }
            }
            GetDiodeADC(DCOFF);

        }
        else if(StartStepB)
        {
            GetDiodeADC(DCINFRA);
            sampleDC = ADCValue[DCINFRA];
            sampleAC = ADCValue[ACINFRA];

            infra.RmsAC  += (long)(sampleAC*sampleAC) >> 8; //Div by 8 makes it easier
            infra.MeanDC += (long)sampleDC >> 8;

            i++;
            if(i == 1000)
            {
                CalculateSP02(&meanSPO2, red.RmsAC, infra.RmsAC);

                LCDWriteStatusValues(meanSPO2, bpm, LCDBattFlag);
                i= 0;
                //Send the Data via SPI.
            }
            firstIterationOver = 1;
        }
    }
} // eof main


#pragma vector = TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void)
{
    TB2CCTL0 &= ~CCIFG;
    StartStepA = 1;
}


#pragma vector = TIMER2_B1_VECTOR
__interrupt void TIMER2_B1_ISR(void)
{
    TB2CTL &= ~TBIFG;
    StartStepB = 1;
}



//Check the maximum
void MinMaxDetection(MinMax* detect, unsigned int sample)
{
    unsigned int check;
    unsigned int* i = &detect->I;
    detect->LastValue[*i] = sample;
    *i++;

    if(*i==4)
    {
        check = detect->LastValue[0];
        if(check > detect->LastValue[1] && check > detect->LastValue[2] && check > detect->LastValue[3] && check > detect->LastValue[4] && detect->Dir == Rising)
        {
            detect->NumOfPeaks++;
            detect->Dir = Falling;
        }
        else if(check < detect->LastValue[1] && check < detect->LastValue[2] && check < detect->LastValue[3] && check < detect->LastValue[4])
        {
            detect->Dir = Rising;
        }
        *i = 0;

    }

}


//Using Active Polling because the combination of Low Power Mode and Interrupt  isn't working properly
void GetDiodeADC(unsigned int channel)
{
    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 |= ADCINCH_3;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);
    ADCValue[channel] = ADCMEM0;

    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 |= ADCINCH_10;
    ADCCTL0  |= ADCENC | ADCSC;

    channel++;

    while(ADCCTL1 & ADCBUSY);
    ADCValue[channel] = ADCMEM0;

}

void GetTemp(unsigned int* meanTemp)
{
    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 |= ADCINCH_11;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);
    ADCValue[TEMP] = ADCMEM0;

    *meanTemp =+ (ADCValue[TEMP] >> 8);
}
void GetBatteryVoltage(unsigned int* lcdBattFlag)
{
    unsigned int voltageLevel = 0;
    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 |= ADCINCH_1;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);

    voltageLevel = ADCMEM0;

    //Based on the battery level, Change a Flag for The LCD

    if(voltageLevel >= 0xF000)
    {
        *lcdBattFlag = BATTERY_FULL; // Full
    }
    else if(voltageLevel >= 0x8000 && voltageLevel < 0xF000)
    {
        *lcdBattFlag = BATTERY_75; // One line missing (~ 75%)
    }
    else if(voltageLevel >= 0x4000 && voltageLevel < 0x8000)
    {
        *lcdBattFlag = BATTERY_50; // Two lines missing (~ 50%)
    }
    else if(voltageLevel >= 0x2000 && voltageLevel < 0x4000)
    {
        *lcdBattFlag = BATTERY_25; // Three lines missing (~ 25%)
    }
    else
    {
        *lcdBattFlag ^= BIT0; // Blinking, BATTERY_10 (0x0)
    }

}


//We will maybe every 100 ms Check for the Intensity
//Maybe even more. Its need to be evaluated during testing.
//
//We need to differentiate when we pull out the finger. This will lead to a maximum, Minum
void CheckLEDIntensity(void)
{
    //Disable both PWM
    TB3CTL &= ~MC_0;

    if(ADCValue[ACRED] < LTresholdRed)
    {
        TB3CCR4++;
        if(TB3CCR4 >= MaxIntensity)
        {
            TB3CCR4 = MaxIntensity;
        }
    }
    else if(ADCValue[ACRED] < HTresholdRed)
    {
        TB3CCR4--;
        if(TB3CCR4 <= 1)
        {
            TB3CCR4 = 1;
        }
    }

    if(ADCValue[ACRED] < LTresholdInfra)
    {
        TB3CCR5++;
        if(TB3CCR5 >= MaxIntensity)
        {
            TB3CCR5 = MaxIntensity;
        }
    }
    else if(ADCValue[ACRED] < HTresholdInfra)
    {
        TB3CCR5--;
        if(TB3CCR5 <= 1)
        {
            TB3CCR5 = 1;
        }
    }


    //Based on the current Level the Threshold gets Adjusted
    //These will be set during
    LTresholdRed = 0;
    HTresholdRed = 0xFFFF;
    LTresholdInfra = 0;
    HTresholdInfra = 0xFFFF;

    //Dont Forget to start the timer again.
    TB3CTL |= MC__UP;
}

void CalculateSP02(unsigned int* meanSPO2, unsigned long rms_AC_Red, unsigned long rms_AC_Infra)
{
    unsigned int R;
    unsigned int SPO2Level;
    //  Possibility 1
    //  R = ((rms_AC_Red / mean_DC_Red)/(rms_AC_Infra/mean_DC_Infra) -0.5) *100; //We have to subtract 0.5 so that we can Access the Array
    //  Possibiltiy 2
    R = log(sqrt(rms_AC_Red))/log(sqrt(rms_AC_Infra)); //when DC_Red =~ DC_Infra

    SPO2Level =  1;
    //SP02Level = SP02[R];

    *meanSPO2 += (SPO2Level >> SPO2Taken); //We take an Average of 4 Values to Calculate the Sp02 Level
}

int CaluclateBPM(int samples)
{
    unsigned int bpm;
    bpm = (SamplesBPM* 60)/(samples/NumOfMaxTaken);
    return bpm;
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

//Writes everything needed that needs to be updated
void LCDWriteStatusValues(unsigned int SP02, unsigned int bpm, unsigned int battStatus)
{


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

