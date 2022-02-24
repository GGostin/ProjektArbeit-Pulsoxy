#include <msp430.h>
#include <math.h>
#include "PCD8544.h"
#include <stdio.h>
#include <string.h>

// LDC PIN DEFINES
#define LCD5110_SCLK_PIN BIT1
#define LCD5110_DN_PIN BIT3
#define LCD5110_SCE_PIN BIT0
#define LCD5110_DC_PIN BIT2
#define LCD5110_SELECT P4OUT &= ~LCD5110_SCE_PIN
#define LCD5110_DESELECT P4OUT |= LCD5110_SCE_PIN
#define LCD5110_SET_COMMAND P4OUT &= ~LCD5110_DC_PIN
#define LCD5110_SET_DATA P4OUT |= LCD5110_DC_PIN
#define LCD5110_COMMAND 0
#define LCD5110_DATA 1

// Defines for normal sequence
#define UPDATELCD 5
#define FULLBATT 4
#define HALFBATT 3
#define MINBATT 2
#define CRITBATT 1

#define MCLK_FREQ_MHZ 4 // MCLK = 4MHz

#define RED 0
#define INFRA 2

typedef enum
{
    DCRED = 0,
    ACRED,
    DCINFRA,
    ACINFRA,
    DCOFF,
    ACOFF,
    TEMP,
    LIPO
} ADCTYPE;
unsigned int ADCValue[8]; //
//unsigned int SPO2Lookup[] = {100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 98, 98, 98, 98, 98, 97, 97, 97, 97, 96, 96, 96, 95, 95, 95, 95, 94, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 91, 90, 90, 90, 89, 89, 89, 88, 88, 88, 87, 87, 87, 86, 86, 86, 85, 85, 84, 84, 84, 83, 83, 83, 82, 82, 81, 81, 81, 80, 80, 79, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 65, 65, 64, 64, 63, 63, 62, 62, 61, 61, 60, 60, 59, 59, 58, 57, 57, 56, 56, 55, 55, 54, 53, 53, 52, 52, 51, 51, 50, 49, 49, 48, 48, 47, 46, 46, 45, 44, 44, 43, 43, 42, 41, 41, 40, 39, 39, 38, 37, 37};
 const int SPO2Lookup[184] = {95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,                              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                              3, 2, 1};
unsigned int StartStepA = 0;
unsigned int StartStepB = 0;

unsigned int UnlockA = 0;
unsigned int UnlockB = 0;

static unsigned int MaxIntensity = 25 * MCLK_FREQ_MHZ;

static unsigned int IntensityChangeRate = 5;
int MaxRedIntens = 60;
int MaxInfraIntens = 60;
int RedIntensity = 60;
int InfraIntensity = 60;

int CalcSpO2Cylce = 250;

unsigned int debugR = 0;
unsigned int debugR2 = 0;
unsigned int intensityLocked;

unsigned int lastBattFlag = 0;

unsigned int lastMinAC = 0;

typedef struct TagMinMax
{
unsigned int maxAC;
unsigned int minAC;
} MinMax;


int delayZeroCrossing = 0;
int ZeroCrossingCounters[100];

typedef enum TagDirection
{
    Rising = 0,
    Falling = 1,
    Undefined = 2
} Direction;

typedef struct TagBPM
{
    unsigned int NumZeroCrossing;
    unsigned int SampleNum;
    unsigned int StartAqui;
    int MinDetectNum;
    unsigned int ValidMinimums;
    unsigned int LastACCheck;
    unsigned int WaitCycles;
    int FirstZeroDetected;
    Direction Dir;
    MinMax Red;
    MinMax Infra;
} BPM;



void ChangeIntensity(unsigned int redDC, unsigned int infraDC)
{
    TB3CTL &= ~MC_0;

    RedIntensity -=IntensityChangeRate;
    if (RedIntensity == IntensityChangeRate)
    {
        RedIntensity = MaxRedIntens;
        InfraIntensity -=IntensityChangeRate;
        if (MaxInfraIntens == IntensityChangeRate)
        {
            InfraIntensity = MaxRedIntens;
        }
    }

    if(abs(redDC - infraDC ) < 2)
    {
        intensityLocked = 1;
        CalcSpO2Cylce = 750;
    }
    TB3CCR4 = RedIntensity;
    TB3CCR5 = InfraIntensity;

    TB3CTL |= MC__UP;
}

void CheckMaximum(BPM *detect, unsigned int counter, unsigned int sampleRed, unsigned int sampleInfra)
{

    unsigned int standardValue = 2000;


    if (counter == 0)
    {
        detect->Red.maxAC = standardValue;
        detect->Red.minAC = standardValue;
        detect->Infra.minAC = standardValue;
        detect->Infra.maxAC = standardValue;
    }
    else
    {
        if (detect->Red.minAC > sampleRed)
        {
            detect->Red.minAC = sampleRed;
        }
        if ( detect->Red.maxAC < sampleRed)
        {
            detect->Red.maxAC = sampleRed;
        }
       if (detect->Infra.minAC > sampleInfra)
            detect->Infra.minAC = sampleInfra;
        if ( detect->Infra.maxAC < sampleInfra)
             detect->Infra.maxAC = sampleInfra;
    }

}

// y(t) = a* x(t) - (1-a) *x(t-1)
unsigned int MovingAverage(unsigned int sample, unsigned int lastSample)
{
    float a = 0.9;
    return (unsigned int)(a * lastSample + (1-a)*sample);

}

void Software_Trim(); // Software Trim to get the best DCOFTRIM value

void InitLCDPins(void);

void GetDiodeADC(unsigned int chann);

void GetTemp(unsigned int *meanTemp);

void GetBatteryVoltage(unsigned int *lcdBattFlag);

int CheckLEDIntensity(int sample);

void CheckZeroCrossing(BPM *detect, unsigned int sample);
int CalculateSPO2(unsigned long rms_AC_RED, unsigned long rms_AC_Infra, unsigned int sampleNum);
int CaluclateBPM(int samples);
void LCDWriteStatusValues(unsigned int SPO2, unsigned int bpm, unsigned int battStatus);
void LCDWriteGraph(unsigned int sample, int row);

unsigned int ConvertADCValueToVoltage(unsigned int value);

void WriteFatNumbers(unsigned int value, unsigned int offset);

void InitPins(void);

void SP02MinMax(BPM detect)
{

    float zaehler = 0, nenner = 0;
    float result = 0;

    zaehler = (float)(detect.Red.maxAC - detect.Red.minAC)/(detect.Infra.maxAC- detect.Infra.minAC);
    result = zaehler * 10000;
    debugR2 = (int)result;
}

void main(void)
{

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    InitPins();

    int counter = 0;
    int errCounter = 0;

    BPM maxDetect = {0};

    long DiffAC = 0;
    unsigned int sampleDC = 0;
    unsigned int sampleAC = 0;
    unsigned int offsetAC = 2100;

    unsigned int lastInfraSample = 0;
    unsigned int lastRedSample = 0;

    unsigned long RmsACRed = 0;
    unsigned long RmsACINfra = 0;
    unsigned int meanBPM = 0;

    unsigned int updLCD = 0;
    unsigned int bpm = 0;
    unsigned int validCounter = 0; // CHeck every second // around 250 Values if current Data is Valid or Invalid, Check is counter > 50 or ErrCounter > 25
    int SpO2= 0;
    int lastSpO2 = 0;

    UnlockA = 1;
    UnlockB = 0;

    int ret = 0;

    unsigned int lcdBattFlag = 0;
    unsigned int meanSPO2 = 0;

    FRCTL0 = FRCTLPW | NWAITS_1;

     __bis_SR_register(SCG0);                                   // disable FLL
     CSCTL3 |= SELREF__REFOCLK;                                 // Set REFO as FLL reference source
     CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_2; // DCOFTRIM=5, DCO Range = 4MHz
     CSCTL2 = FLLD_0 + 120;                                     // DCOCLKDIV = 4MHz
     __delay_cycles(3);
     __bic_SR_register(SCG0); // enable FLL
     Software_Trim();         // Software Trim to get the best DCOFTRIM value

     CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
    //                                            // default DCOCLKDIV as MCLK and SMCLK source

    // Configuration for ADC Pin start on DC for RED (A3, P1.3)
    // Later each ADC Pin (A1, A3, A10, A11) is configured and triggered manually

    P1SEL0 |= BIT1 | BIT3;
    P1SEL1 |= BIT1 | BIT3;
    P5SEL0 |= BIT2 | BIT3;
    P5SEL1 |= BIT2 | BIT3;

    ADCCTL0 &= ~ADCENC;
    ADCCTL0 |= ADCON;      // ADC-Core on
    ADCCTL1 |= ADCSHP;     // This just makes it so when you start the conversion it stops eventually.
    ADCCTL2 &= ~ADCRES;    // Reset Bit Conversion
    ADCCTL2 |= ADCRES_2;   // 12 Bit Conversion
    ADCMCTL0 |= ADCINCH_3; // Channel A3
    // ADCIE |= ADCIE0;

    InitLCDPins();
    __delay_cycles(600000 * MCLK_FREQ_MHZ);

    // Use PIn 5.0 and 5.1 for switching the LEDs
    P5DIR |= BIT0 | BIT1;
    P5SEL0 |= BIT0 | BIT1;

    TB2CCR0 = (2000 - 1) * MCLK_FREQ_MHZ; // Every 2 ms  the LED is already on for 500 us and will continue for another 500 us
    TB2CCTL1 |= OUTMOD_2;                 // TB2CCR1 toggle/set
    TB2CCR1 = (500 - 1) * MCLK_FREQ_MHZ;  // 0.500us , 3.5 ms ... 4ms InfraRed is on
    TB2CCTL2 |= OUTMOD_6;                 // TBCCR2 reset/set
    TB2CCR2 = (1500 - 1) * MCLK_FREQ_MHZ; // 1.5 ms - 2.5 ms Red is on
    TB2CTL |= TBSSEL_2 | MC_3 | TBCLR;    // SMCLK, Up-Down-Mode, Interrupt Enable on Max Value and Zero
    TB2CTL |= TBIE;
    TB2CCTL0 |= CCIE; // Enable Interrupt when CCR0 is reached.

    // Two PWM for Intensity
    // P6.3 and P6.4

    // P2DIR |= BIT5;
    // P2OUT |= BIT5;

    P6DIR |= BIT3 | BIT4;
    P6SEL0 |= BIT3 | BIT4;

    TB3CCR0 = MaxIntensity;
    TB3CCTL4 |= OUTMOD_7;
    TB3CCTL5 |= OUTMOD_7;
    TB3CCR4 = RedIntensity;
    TB3CCR5 = InfraIntensity;
    TB3CTL |= TBSSEL__SMCLK | MC__UP | TBCLR; // Up-Mode

    PM5CTL0 &= ~LOCKLPM5; // Without this the pins won't be configured in Hardware.
    TB2CTL &= ~TBIFG;
    TB2CCTL0 &= ~CCIFG;
    initLCD();
    clearLCD();

    setAddr(8, 0);
    writeStringToLCD("SPO2");

    setAddr(60, 0);
    writeStringToLCD("bpm");

    clearBank(4);
    setAddr(38, 4);
    writeBattery(BATTERY_FULL);

    // StartAqui a second for everything to start
    __delay_cycles(1000000 * MCLK_FREQ_MHZ);

    __bis_SR_register(GIE);

    while (1)
    {

        if (StartStepA)
        {

            GetDiodeADC(RED);
            sampleDC = ADCValue[RED];
            sampleAC = ADCValue[RED + 1];
            ret = CheckLEDIntensity(sampleAC);

            if (!ret)
            {
                errCounter++;
                counter = 0;
                memset(&maxDetect, 0, sizeof(maxDetect));
                maxDetect.WaitCycles = 125;
                RmsACRed = 0;
                RmsACINfra = 0;

                lastInfraSample = 0;
                lastRedSample = 0;
            }


            CheckMaximum(&maxDetect,counter, ADCValue[RED+1], ADCValue[INFRA+1]);

            CheckZeroCrossing(&maxDetect, sampleAC);

            if (counter == 100)
            {
                GetBatteryVoltage(&lcdBattFlag);
            }

            if (counter == CalcSpO2Cylce)
            {
                if(!intensityLocked)
                {
                    ChangeIntensity(ADCValue[DCRED],ADCValue[DCINFRA]);
                }
            }


            if (maxDetect.NumZeroCrossing == 7)
            {

                bpm = CaluclateBPM(maxDetect.SampleNum);
                updLCD = 1;
                errCounter = 0;

                SpO2 =  CalculateSPO2(RmsACRed, RmsACINfra, counter);

                if(meanBPM != 0)
                {
                    meanBPM = (meanBPM + bpm)/2;
                }
                else{
                    meanBPM = bpm;
                }

            }

            // Take RMS of AC and Mean of DC


            sampleAC = MovingAverage(sampleAC,lastRedSample);
            DiffAC = (long)((long)sampleAC - offsetAC);

            lastRedSample = sampleAC;

            if(maxDetect.FirstZeroDetected == 1)
            {
                RmsACRed += (DiffAC * DiffAC);
            }

            UnlockA = 0;
            UnlockB = 1;
            StartStepA = 0;

        }
        else if (StartStepB)
        {

            GetDiodeADC(INFRA);
            sampleDC = ADCValue[INFRA];
            sampleAC = ADCValue[INFRA + 1];
            sampleAC = MovingAverage(sampleAC,lastInfraSample);
            DiffAC = (long)((long)sampleAC - offsetAC);
            lastInfraSample = sampleAC;

            if(maxDetect.FirstZeroDetected == 1)
            {
               RmsACINfra += (DiffAC * DiffAC);
            }


            counter++;
            validCounter++;
            if (errCounter == 1000)
            {
                clearBank(2);
                clearBank(3);
                setAddr(0, 2);
                writeStringToLCD("Invalid Signal");
                errCounter = 0;
            }

            if (updLCD)
            {
                char text[6];
                clearBank(5);
                setAddr(0, 5);
                sprintf(text,"%d",debugR);
                writeStringToLCD("R: ");
                writeStringToLCD(text);
               // writeStringToLCD(" Data Valid");
                LCDWriteStatusValues(SpO2, bpm, lcdBattFlag);

                updLCD = 0;
                counter = 0;

                lastInfraSample = 0;
                lastRedSample = 0;

                memset(&maxDetect, 0, sizeof(maxDetect));
                maxDetect.WaitCycles = 250;
            }

            if (validCounter == 250)
            {
                setAddr(0, 5);
                if (errCounter >= 250)
                {
                  //  writeStringToLCD("Data Invalid");
                }
                else
                {
                   // clearBank(5);
                }
                validCounter = 0;
            }

            UnlockA = 1;
            UnlockB = 0;
            StartStepB = 0;


        }
    }
    //__bis_SR_register(LPM0);
} // eof main

#pragma vector = TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void)
{
    TB2CCTL0 &= ~CCIFG;

    if (UnlockB == 1)
    {
        StartStepB = 1;
    }


}

#pragma vector = TIMER2_B1_VECTOR
__interrupt void TIMER2_B1_ISR(void)
{
    TB2CTL &= ~TBIFG;

    if (UnlockA == 1)
    {
        StartStepA = 1;
    }
    // P1OUT |= BIT0;
}

// Sets a defined State for all Pins (High Ohm)
// Afterwards the selected individually
void InitPins(void)
{
    P1DIR = 0xFF;
    P1OUT = 0;
    P2DIR = 0xFF;
    P2OUT = 0;
    P3DIR = 0xFF;
    P3OUT = 0;
    P4DIR = 0xFF;
    P4OUT = 0;
    P5DIR = 0xFF;
    P5OUT = 0;
    P6DIR = 0xFF;
    P6OUT = 0;
    PADIR = 0xFF;
    PAOUT = 0;
    PBDIR = 0xFF;
    PBOUT = 0;
    PCDIR = 0xFF;
    PCOUT = 0;
}

// Check the maximum
void CheckZeroCrossing(BPM *detect, unsigned int sample)
{

    // The AC Value floats around the DC Value of 1.5 V ~ 1930
    int diff = abs(sample - 2100);

    if(detect->FirstZeroDetected == 1)
    {
        detect->SampleNum++;
    }

    if (detect->WaitCycles == 0 )
    {
        if (diff < 40 )
        {
            detect->NumZeroCrossing++;
            detect->FirstZeroDetected = 1;
            detect->WaitCycles = 50;
        }
    }
    else{
        detect->WaitCycles--;
    }

}

// Using Active Polling because the combination of Low Power Mode and Interrupt  isn't working properly
void GetDiodeADC(unsigned int channel)
{
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; // Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_3;
    ADCCTL0 |= ADCENC | ADCSC;

    while (ADCCTL1 & ADCBUSY)
        ;
    ADCValue[channel] = ADCMEM0;

    channel++;
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; // Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_10;
    ADCCTL0 |= ADCENC | ADCSC;

    while (ADCCTL1 & ADCBUSY)
        ;
    ADCValue[channel] = ADCMEM0;
}
unsigned int ConvertADCValueToVoltage(unsigned int value)
{
    // Delta is 3.3V/2^12 or arund 8 * 10^4
    // We approx by multiplying with 8 and and shiftign with 8192
    // To make it better comparable we multiply it by 10.
    unsigned int approxValue = value * 12;
    approxValue = approxValue >> 10;
    return approxValue;
}

void GetTemp(unsigned int *meanTemp)
{
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; // Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_11;
    ADCCTL0 |= ADCENC | ADCSC;

    while (ADCCTL1 & ADCBUSY)
        ;
    ADCValue[TEMP] = ConvertADCValueToVoltage(ADCMEM0);

    *meanTemp = +(ADCValue[TEMP] >> 8);
}
void GetBatteryVoltage(unsigned int *lcdBattFlag)
{
    unsigned int voltageLevel = 0;
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ADCINCH_0; // Reset the channel for next Conversion
    ADCMCTL0 |= ADCINCH_1;
    ADCCTL0 |= ADCENC | ADCSC;

    while (ADCCTL1 & ADCBUSY)
        ;
    voltageLevel = ConvertADCValueToVoltage(ADCMEM0);

    // Based on the battery level, Change a Flag for The LCD

    if (voltageLevel >= 33)
    {
        *lcdBattFlag = BATTERY_FULL; // Full
    }
    else if (voltageLevel >= 24 && voltageLevel < 33)
    {
        *lcdBattFlag = BATTERY_75; // One line missing (~ 75%)
    }
    else if (voltageLevel >= 22 && voltageLevel < 24)
    {
        *lcdBattFlag = BATTERY_50; // Two lines missing (~ 50%)
    }
    else if (voltageLevel >= 11 && voltageLevel < 22)
    {
        *lcdBattFlag = BATTERY_25; // Three lines missing (~ 25%)
    }
    else
    {
        *lcdBattFlag ^= BIT0; // Blinking, BATTERY_10 (0x0)
    }
}

// We will maybe every 100 ms Check for the Intensity
// Maybe even more. Its need to be evaluated during testing.
//
// We need to differentiate when we pull out the finger. This will lead to a maximum, Minum
int CheckLEDIntensity(int sample)
{
    // In Range  2,7 V to 0,3 V;
    if (sample > 3350 || sample < 370)
    {
        return 0;
    }

    return 1;
}

int CalculateSPO2(unsigned long rms_AC_Red, unsigned long rms_AC_Infra, unsigned int sampleNum)
{
    unsigned int R;

    float zaehler = 0, nenner = 0;
    float fR = 0;
    int SPO2Level = 0;

    if (rms_AC_Infra != 0)
    {
        zaehler = log(sqrt(rms_AC_Red/sampleNum));
        nenner = log(sqrt(rms_AC_Infra/sampleNum));
        fR = zaehler/nenner;

        if(fR > 1) fR = fR-1;
        else fR = 1 - fR;

        R = fR * 100000; // when DC_Red =~ DC_Infra
        debugR = (int)(fR * 100000*100);
        if (R >= (sizeof(SPO2Lookup) / sizeof(SPO2Lookup[0])))
        {
            return 0;
        }

        SPO2Level = SPO2Lookup[R];
    }
    return SPO2Level;
}

int CaluclateBPM(int samples)
{
    const unsigned int samplesBPM = 250; // Per 1 sec we take roughly 250 samples of the red LED
    unsigned int numOfMaxTaken =3;

    unsigned int bpm;

    bpm = (samplesBPM * 60 * numOfMaxTaken) / samples;
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

    UCA1CTLW0 |= UCSWRST;                         // **Put state machine in reset**
    UCA1CTLW0 |= UCMST | UCSYNC | UCCKPH | UCMSB; // 3-pin, 8-bit SPI master
    UCA1CTLW0 |= UCSSEL_3;                        // SMCLK
    UCA1BRW |= 0;                                 // 1:1

    UCA1CTLW0 &= ~UCSWRST;
}

// Writes everything needed that needs to be updated
void LCDWriteStatusValues(unsigned int SPO2, unsigned int bpm, unsigned int battStatus)
{

    clearBank(2);
    clearBank(3);
    WriteFatNumbers(SPO2, 0);
    WriteFatNumbers(bpm, 42);

    if (lastBattFlag = !battStatus)
    {
        setAddr(38, 4);
        writeBattery(battStatus);
    }

    lastBattFlag = battStatus;
}

void setAddr(unsigned char xAddr, unsigned char yAddr)
{
    writeToLCD(LCD5110_COMMAND, PCD8544_SETXADDR | xAddr);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETYADDR | yAddr);
}

void writeToLCD(unsigned char dataCommand, unsigned char data)
{
    LCD5110_SELECT;

    if (dataCommand)
    {
        LCD5110_SET_DATA;
    }
    else
    {
        LCD5110_SET_COMMAND;
    }

    UCA1TXBUF = data;
    while (!(UCTXIFG & UCA1IFG))

        LCD5110_DESELECT;
}

void initLCD()
{
    writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETVOP | 0x3F);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETTEMP | 0x02);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETBIAS | 0x03);
    writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET);
    writeToLCD(LCD5110_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

void writeCharToLCD(char c)
{
    unsigned char i;
    for (i = 0; i < 5; i++)
    {
        writeToLCD(LCD5110_DATA, font[c - 0x20][i]);
    }
    writeToLCD(LCD5110_DATA, 0);
}

void writeStringToLCD(const char *string)
{
    while (*string)
    {
        writeCharToLCD(*string++);
    }
}

void writeBattery(const int i)
{
    unsigned char j;
    for (j = 0; j < 7; j++)
    {
        writeToLCD(LCD5110_DATA, Battery[i][j]);
    }
    writeToLCD(LCD5110_DATA, 0);
}

void clearLCD()
{
    setAddr(0, 0);
    int i = 0;
    while (i < PCD8544_MAXBYTES)
    {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, 0);
}

void clearBank(unsigned char bank)
{
    setAddr(0, bank);
    int i = 0;
    while (i < PCD8544_HPIXELS)
    {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, bank);
}

void WriteFatNumbers(unsigned int value, unsigned int offset)
{
    unsigned int num[3]; // 103 => num[0]=1 , num[1]=0, num[2]=3
    unsigned char numValid[3];
    unsigned int firstColumn = 10 + offset; // StartColumn for two Numbers
    unsigned int i, j;

    if (value >= 100)
    {
        num[0] = 1;
        num[1] = (value - 100) / 10;
        num[2] = (value % 10);
        firstColumn -= 4; // StartColum gets reduced to 6 far left and right.
    }
    else if (value >= 10 && value < 100)
    {
        num[0] = 0;
        num[1] = (value / 10);
        num[2] = (value % 10);
    }
    else
    {
        num[0] = 0;
        num[1] = 0;
        num[2] = value;
        firstColumn += 7;
    }
    numValid[0] = num[0] > 0 ? 1 : 0;
    if (num[1] > 0 || (num[1] == 0 && num[0] != 0))
        numValid[1] = 1;
    if (num[2] > 0 || (num[1] == 0 || num[0] == 0) && num[2] == 0)
        numValid[2] = 1;

    // First write Upper Number Part
    setAddr(firstColumn, 2);

    for (j = 0; j < 3; j++)
    {
        if (numValid[j])
        {

            for (i = 0; i < 10; i++)
            {
                writeToLCD(LCD5110_DATA, FatNumber_Upper[num[j]][i]);
            }
            // Two Spaces between Numbers
            writeToLCD(LCD5110_DATA, 0);
            writeToLCD(LCD5110_DATA, 0);
        }
    }
    // For Second Iteration write Lower Numbers
    setAddr(firstColumn, 3);
    for (j = 0; j < 3; j++)
    {
        if (numValid[j])
        {
            for (i = 0; i < 10; i++)
            {
                writeToLCD(LCD5110_DATA, FatNumber_Lower[num[j]][i]);
            }
            // Two Spaces between Numbers
            writeToLCD(LCD5110_DATA, 0);
            writeToLCD(LCD5110_DATA, 0);
        }
    }
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
        CSCTL0 = 0x100; // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;     // Clear DCO fault flag
        } while (CSCTL7 & DCOFFG); // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ); // Wait FLL lock status (FLLUNLOCK) to be stable
                                                            // Suggest to wait 24 cycles of divided FLL reference clock
        while ((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0))
            ;

        csCtl0Read = CSCTL0; // Read CSCTL0
        csCtl1Read = CSCTL1; // Read CSCTL1

        oldDcoTap = newDcoTap;                    // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;          // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070) >> 4; // Get DCOFTRIM value

        if (newDcoTap < 256) // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;                   // Delta value between DCPTAP and 256
            if ((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                                 // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim << 4);
            }
        }
        else // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256; // Delta value between DCPTAP and 256
            if (oldDcoTap < 256)           // DCOTAP cross 256
                endLoop = 1;               // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim << 4);
            }
        }

        if (newDcoDelta < bestDcoDelta) // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    } while (endLoop == 0); // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy; // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy; // Reload locked DCOFTRIM
    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}
