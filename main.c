#include <msp430.h>
#include <math.h>
#include "PCD8544.h"
#include <stdio.h>

//LDC PIN DEFINES
#define LCD5110_SCLK_PIN            BIT1
#define LCD5110_DN_PIN              BIT3
#define LCD5110_SCE_PIN             BIT0
#define LCD5110_DC_PIN              BIT2
#define LCD5110_SELECT              P4OUT &= ~LCD5110_SCE_PIN
#define LCD5110_DESELECT            P4OUT |= LCD5110_SCE_PIN
#define LCD5110_SET_COMMAND         P4OUT &= ~LCD5110_DC_PIN
#define LCD5110_SET_DATA            P4OUT |= LCD5110_DC_PIN
#define LCD5110_COMMAND             0
#define LCD5110_DATA                1

//Defines for normal sequence
#define UPDATELCD   5
#define FULLBATT    4
#define HALFBATT    3
#define MINBATT     2
#define CRITBATT    1

#define DATAVALID   6
#define DATAWAIT    7

#define CHANGEINTENSITY 1
#define KEEPINTENSITY   0

#define MCLK_FREQ_MHZ 16                     // MCLK = 16MHz

#define RED 0
#define INFRA 2


typedef enum{ DCRED = 0, ACRED, DCINFRA, ACINFRA, DCOFF, ACOFF, TEMP, LIPO}ADCTYPE;
unsigned int ADCValue[8];//
unsigned int SPO2Lookup[] = {100, 100,100, 100, 100, 100, 100, 100, 99 , 99 , 99 , 99 , 98 , 98  ,98 , 98,  98  ,97 , 97  ,97, 97,  96  ,96 , 96,  95  ,95 , 95,  95  ,94 , 94,  94,  94,  93,  93,  93,  92,  92,  92,  91,  91,  91,  90,  90,  90,  89,  89,  89,  88,  88,  88,  87,  87,  87 ,86 , 86,  86  ,85 , 85,  84  ,84 , 84,  83  ,83 , 83,  82  ,82 , 81,  81  ,81 , 80,  80  ,79 , 79,  79  ,78 , 78,  77,  77, 76 , 76 , 75 , 75 , 75 , 74 , 74 , 73 , 73 , 72 , 72 , 71 , 71 , 70 , 70 , 69 , 69 , 68 , 68 , 67 , 67 , 66 , 66 , 65 , 65 , 64 , 64 , 63 , 63 , 62 , 62 , 61 , 61 , 60 , 60 , 59 , 59 , 58 , 57 ,57  ,56  ,56  ,55  ,55  ,54  ,53  ,53  ,52  ,52  ,51  ,51  ,50  ,49  ,49  ,48  ,48  ,47  ,46  ,46  ,45  ,44  ,44  ,43  ,43  ,42  ,41  ,41  ,40  ,39  ,39  ,38  ,37  ,37};


unsigned int WaitCalcSP02 = 0;

unsigned int StartStepA   = 0;
unsigned int StartStepB   = 0;

static unsigned int MaxIntensity = 10;


//Question is. Do both need to be changed?
//I could argue that one is enough because the other will probably be around the same value
const static unsigned int HTresholdRed = 0x0FFF;
const static unsigned int LTresholdRed = 0;
const static unsigned int HTresholdInfra = 0x0FFF;
const static unsigned int LTresholdInfra = 0;


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

void Software_Trim();                        // Software Trim to get the best DCOFTRIM value


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
int CheckLEDIntensity(int changeValues);

void MinMaxDetection(MinMax* detect, unsigned int sample);
void CalculateSP02(unsigned int* meanSPO2, unsigned long rms_AC_RED, unsigned long rms_AC_Infra);
int CaluclateBPM(int samples);
void LCDWriteStatusValues(unsigned int SPO2, unsigned int bpm, unsigned int battStatus);
void LCDWriteGraph(unsigned int sample, int row);

unsigned int ConvertADCValueToVoltage(unsigned int value);

void InitPins(void);

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    InitPins();

    MinMax maxDetect = {0};
    maxDetect.Dir = Falling;

    unsigned int sampleDC = 0;
    unsigned int sampleAC = 0;

    unsigned int i = 0;
    unsigned int samplesHBeat = 0;
    unsigned int updLCD = 0;
    unsigned int bpm =0;
    unsigned int firstIterationOver = 0;
    unsigned int changeIntensity = CHANGEINTENSITY;
    unsigned int iterIntensity = 0;

    int ret = 0;

    CSCTL1 |= DCORSEL_5;

    //##TestPIN
    P1DIR |= BIT0;
    //##TestPIN


    LEDValues red;
    red.MeanDC = 0;
    red.RmsAC = 0;
    LEDValues infra;
    infra.MeanDC = 0;
    infra.RmsAC = 0;

    unsigned int meanTemp = 0;
    unsigned int lcdBattFlag = 0;
    unsigned int meanSPO2 = 0;


    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_5;// DCOFTRIM=5, DCO Range = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    Software_Trim();                                   // Software Trim to get the best DCOFTRIM value

     CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;        // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                       // default DCOCLKDIV as MCLK and SMCLK source

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
    __delay_cycles(300000*MCLK_FREQ_MHZ); // we have to wait a bit for the LCD to boot up.
    initLCD();
    clearLCD();


    //Use PIn 5.0 and 5.1 for switching the LEDs
    P5DIR |= BIT0 | BIT1;
    P5SEL0 |= BIT0 | BIT1;

    TB2CCR0  = (2000-1) *MCLK_FREQ_MHZ;          //Every 2 ms  the LED is already on for 500 us and will continue for another 500 us
    TB2CCTL1 |= OUTMOD_2;    //TB2CCR1 toggle/set
    TB2CCR1  = (500-1)*MCLK_FREQ_MHZ;         //0.500us , 3.5 ms ... 4ms
    TB2CCTL2 |= OUTMOD_6;    //TBCCR2 reset/set
    TB2CCR2  = (1500-1)*MCLK_FREQ_MHZ;         //1.5 ms - 2.5 ms
    TB2CTL   |= TBSSEL_2 | MC_3 |TBCLR; // SMCLK, Up-Down-Mode, Interrupt Enable on Max Value and Zero
    TB2CTL   |= TBIE;
    TB2CCTL0 |= CCIE; //Enable Interrupt when CCR0 is reached.

    //Two PWM for Intensity
    //P6.3 and P6.4

    P6DIR  |= BIT3 | BIT4;
    P6SEL0 |= BIT3 | BIT4;

    TB3CCR0   = MaxIntensity*10;
    TB3CCTL4 |= OUTMOD_7;
    TB3CCTL5 |= OUTMOD_7;
    TB3CCR4   = (MaxIntensity>>1)*10;
    TB3CCR5   = (MaxIntensity>>1)*10;
    TB3CTL   |= TBSSEL__SMCLK  | MC__UP | TBCLR; //Up-Mode

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

    //Wait a second for everything to start
    __delay_cycles(1000000*MCLK_FREQ_MHZ);

    __bis_SR_register(GIE);


    while(1)
    {
        if(StartStepA)
        {
            P1OUT |= BIT0;
            GetDiodeADC(RED);

            //Take RMS of AC and Mean of DC
            sampleDC = ADCValue[RED];
            sampleAC = ADCValue[RED];

            red.RmsAC = ((unsigned long)sampleAC)*sampleAC; //Div by 8 makes it easier
            red.RmsAC += red.RmsAC >>8;
            red.MeanDC += (unsigned long)sampleDC >> 8;

            GetTemp(&meanTemp);

            ret = CheckLEDIntensity(changeIntensity);

            if(firstIterationOver && ret == DATAVALID)
            {
                samplesHBeat++;
                MinMaxDetection(&maxDetect, sampleAC);

                if (i == 100)
                {
                    GetBatteryVoltage(&lcdBattFlag);
                }

                if(maxDetect.NumOfPeaks == 5)
                {
                    bpm = CaluclateBPM(samplesHBeat);
                    samplesHBeat = 0;
                    maxDetect.NumOfPeaks = 0;
                }

                if(i == 250)
                {
                    i = 0;
                    updLCD = 1;
                    CalculateSP02(&meanSPO2, red.RmsAC, infra.RmsAC);
                    //meanSPO2++;
                }

            }
            else
            {
                //We wait for like 100 ms and check again, this here is called every 2 ms so we wait 50 Cycles;
                changeIntensity = KEEPINTENSITY;
                iterIntensity++;
                if(iterIntensity == 50)
                {
                    changeIntensity = CHANGEINTENSITY;
                }
            }
            //Do we need this ??
            //GetDiodeADC(DCOFF);

            P1OUT &= ~BIT0;
            StartStepA = 0;

        }
        else if(StartStepB)
        {

            GetDiodeADC(INFRA);
            sampleDC = ADCValue[INFRA];
            sampleAC = ADCValue[INFRA];

            infra.RmsAC  += (long)(sampleAC*sampleAC) >> 8; //Div by 8 makes it easier
            infra.MeanDC += (long)sampleDC >> 8;

            i++;
            if(updLCD)
            {
                LCDWriteStatusValues(meanSPO2, bpm, lcdBattFlag);
                updLCD = 0;
                red.MeanDC  = 0;
                red.RmsAC   = 0;
                red.MeanDC  = 0;
                infra.RmsAC = 0;
                meanSPO2 = 0;
                //Send the Data via SPI.
            }
            firstIterationOver = 1;

            //P1OUT &= ~BIT0;
            StartStepB = 0;

        }
    }
    //__bis_SR_register(LPM0);
} // eof main


#pragma vector = TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void)
{
    TB2CCTL0 &= ~CCIFG;

    if(!WaitCalcSP02)
    {
        StartStepA = 1;
    }

    //_bic_SR_register_on_exit(LPM0);
}


#pragma vector = TIMER2_B1_VECTOR
__interrupt void TIMER2_B1_ISR(void)
{
    TB2CTL &= ~TBIFG;
    if(!WaitCalcSP02)
    {
    StartStepB = 1;
    }
    //P1OUT |= BIT0;
    //_bic_SR_register_on_exit(LPM0);
}


//Sets a defined State for all Pins (High Ohm)
//Afterwards the selected individually
void InitPins(void)
{
    P1DIR = 0xFF; P1OUT = 0;
    P2DIR = 0xFF; P2OUT = 0;
    P3DIR = 0xFF; P3OUT = 0;
    P4DIR = 0xFF; P4OUT = 0;
    P5DIR = 0xFF; P5OUT = 0;
    P6DIR = 0xFF; P6OUT = 0;
    PADIR = 0xFF; PAOUT = 0;
    PBDIR = 0xFF; PBOUT = 0;
    PCDIR = 0xFF; PCOUT = 0;
}

//Check the maximum
void MinMaxDetection(MinMax* detect, unsigned int sample)
{
    unsigned int check;
    unsigned int* i = &detect->I;
    detect->LastValue[*i] = sample;
    (*i)++;

    if(*i==5)
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
    ADCMCTL0 &= ADCINCH_0; //Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_3;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);
    ADCValue[channel] = ADCMEM0;

    channel++;
    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; //Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_10;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);
    ADCValue[channel] = ADCMEM0;

}
unsigned int ConvertADCValueToVoltage(unsigned int value)
{
    //Delta is 3.3V/2^12 or arund 8 * 10^4
    //We approx by multiplying with 8 and and shiftign with 8192
    //To make it better comparable we multiply it by 10.
    unsigned int approxValue = value * 12;
    approxValue = approxValue >> 10;
    return approxValue;

}

void GetTemp(unsigned int* meanTemp)
{
    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; //Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_11;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);
    ADCValue[TEMP] = ConvertADCValueToVoltage(ADCMEM0);


    *meanTemp =+ (ADCValue[TEMP] >> 8);
}
void GetBatteryVoltage(unsigned int* lcdBattFlag)
{
    unsigned int voltageLevel = 0;
    ADCCTL0  &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; //Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_1;
    ADCCTL0  |= ADCENC | ADCSC;

    while(ADCCTL1 & ADCBUSY);
    voltageLevel = ConvertADCValueToVoltage(ADCMEM0);

    //Based on the battery level, Change a Flag for The LCD

    if(voltageLevel >= 33)
    {
        *lcdBattFlag = BATTERY_FULL; // Full
    }
    else if(voltageLevel >= 24 && voltageLevel < 33)
    {
        *lcdBattFlag = BATTERY_75; // One line missing (~ 75%)
    }
    else if(voltageLevel >= 22 && voltageLevel < 24)
    {
        *lcdBattFlag = BATTERY_50; // Two lines missing (~ 50%)
    }
    else if(voltageLevel >= 11 && voltageLevel < 22)
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
int CheckLEDIntensity(int changeValues)
{

    int ret = DATAVALID;


    if(changeValues)
    {
        //Disable both PWM
        TB3CTL &= ~MC_0;
    }

    if(ADCValue[ACRED] < LTresholdRed)
    {
        ret = DATAWAIT;

        if(changeValues)
        {
            TB3CCR4++;
            if(TB3CCR4 >= MaxIntensity)
            {
                TB3CCR4 = MaxIntensity;
            }
        }
    }
    else if(ADCValue[ACRED] > HTresholdRed)
    {
        ret = DATAWAIT;
        if(changeValues)
        {
            TB3CCR4--;
            if(TB3CCR4 <= 1)
            {
                TB3CCR4 = 1;
            }
        }
    }

    if(ADCValue[ACRED] < LTresholdInfra)
    {
        ret = DATAWAIT;

        if(changeValues)
        {
            TB3CCR5++;
            if(TB3CCR5 >= MaxIntensity)
            {
                TB3CCR5 = MaxIntensity;
            }
        }
    }
    else if(ADCValue[ACRED] > HTresholdInfra)
    {
        ret = DATAWAIT;

        if(changeValues)
        {
            TB3CCR5--;
            if(TB3CCR5 <= 1)
            {
                TB3CCR5 = 1;
            }
        }
    }

    //Dont Forget to start the timer again.
    if(changeValues)
    {
        TB3CTL |= MC__UP;
    }
    return ret;
}

void CalculateSP02(unsigned int* meanSPO2, unsigned long rms_AC_Red, unsigned long rms_AC_Infra)
{
    unsigned int R;
    const unsigned int SPO2Taken = 2; //Right Shift
    unsigned int SPO2Level;

    WaitCalcSP02 = 1;

    if(rms_AC_Infra != 0)
    {
        R = log((rms_AC_Red))/log((rms_AC_Infra))*100; //when DC_Red =~ DC_Infra

        if(R >= (sizeof(SPO2Lookup)/sizeof(SPO2Lookup[0])))
        {
            WaitCalcSP02 = 0;
            return;
        }

        SPO2Level = SPO2Lookup[R];

        *meanSPO2 += (SPO2Level >> SPO2Taken); //We take an Average of 4 Values to Calculate the Sp02 Level
    }

    WaitCalcSP02 = 0;
}

int CaluclateBPM(int samples)
{
    const unsigned int samplesBPM = 250; //Per 1 sec we take roughly 250 samples of the red LED
    unsigned int numOfMaxTaken = 4;
;
    unsigned int bpm;


    bpm = (samplesBPM* 60)/(samples/numOfMaxTaken);
    return bpm;
}


void InitLCDPins(void)
{
    P4SEL0 |= LCD5110_DN_PIN | LCD5110_SCLK_PIN;

    P6DIR |= BIT0 | BIT1;
    P6OUT |= BIT0 | BIT1;

    P4OUT |= LCD5110_SCE_PIN;
    P4DIR |= LCD5110_SCE_PIN;

    P4OUT |= LCD5110_DC_PIN;
    P4DIR |= LCD5110_DC_PIN;

    UCA1CTLW0 |= UCSWRST;                     // **Put state machine in reset**
    UCA1CTLW0 |= UCMST|UCSYNC|UCCKPH|UCMSB;   // 3-pin, 8-bit SPI master
    UCA1CTLW0 |= UCSSEL_3; // SMCLK
    UCA1BRW   |= 0; //1:1

    UCA1CTLW0 &= ~UCSWRST;
}

//Just do it every 100ms
//Based on the current sample we draw it to the 5 row.
//We have only 8 Pixels  per Row
void LCDWriteGraph(unsigned int sample, int row)
{




}

//Writes everything needed that needs to be updated
void LCDWriteStatusValues(unsigned int SPO2, unsigned int bpm, unsigned int battStatus)
{
    char spo2[4];
    char sbpm[4];

    sprintf(spo2,"%d",SPO2);
    sprintf(sbpm,"%d", bpm);

    clearBank(3);
    clearBank(4);

    setAddr(3,24);
    writeStringToLCD(spo2);
    setAddr(42,24);
    writeStringToLCD(sbpm);

    setAddr(0,48);
    writeBattery(battStatus);
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

    UCA1TXBUF  = data;
    while(!(UCTXIFG & UCA1IFG ))

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

void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}
