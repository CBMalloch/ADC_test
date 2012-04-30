/*******************************************************************************
  ADC test

 
*/

// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************

#include <p32xxxx.h>
#include <project_setup.h>
#include <utility.h>
#include <GenericTypeDefs.h>
#include <plib.h>

// *****************************************************************************
// *****************************************************************************
// Section: Configuration bits
// *****************************************************************************
// *****************************************************************************

#pragma config POSCMOD = OFF      // primary oscillator conf
#pragma config OSCIOFNC = OFF     // CLKO disconnect from output pin
#pragma config FSOSCEN = OFF      // secondary oscillator disable

// FNOSC FRCPLL selects FRC for input to PLL as well as PLL output for SYSCLK
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_2  // PLL 40MHz
#pragma config FNOSC = FRCPLL       // oscillator selection
#pragma config FPBDIV = DIV_1     // peripheral bus clock divisor 40MHz

#pragma config FWDTEN = OFF       // watchdog timer
#pragma config FCKSM = CSECME     // clock switching and monitor selection

#pragma config CP = OFF           // code (read and modify) protect
#pragma config BWP = OFF          // boot flash write-protect
#pragma config PWP = OFF          // program flash write-protect

// #pragma config ICESEL = ICS_PGx1  // ice/icd comm channel select
#pragma config JTAGEN = OFF       // JTAG disable
// JTAG port pins are multiplexed with PortA pins RA0, RA1, RA4, and RA5

// *****************************************************************************
// *****************************************************************************
// Section: System Macros
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Constant Data
// *****************************************************************************
// *****************************************************************************

volatile int programStatus;
int clock_interrupt_period_us = 1000;
INT8 enableADC   = 0;
INT8 enablePrint = 0;
#define LED_blink_period_ms  200
#define ADC_interval_ms        5
#define print_interval_ms    500

// *****************************************************************************
// *****************************************************************************
// Section: Code
// *****************************************************************************
// *****************************************************************************

int main(void)
{
  UINT8   strBuf[bufLen];
  UINT8   bank = 0;
  #define NSENSORSINBANK 4
  UINT32  cPot, cSensor[2][NSENSORSINBANK];
  UINT8   i;
  
  LED_TRIS = 1;
  
  PWR_BANK0_TRIS = 0;  // disable both banks to start
 
  PWR_BANK1_TRIS = 1;

  UART2_RX_PPS_REG = UART2_RX_PPS_ITM;
  UART2_TX_PPS_REG = UART2_TX_PPS_ITM;

  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
  UARTSetDataRate(UART2, FPB, 19200);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // timer interrupt stuff...

  // configure the core timer roll-over rate
  // the internal source is the Peripheral Clock
  // timer_preset = SYSCLK_freq / pb_div / additional prescale / desired_ticks_per_sec

  UINT16 timer_preset = (clock_interrupt_period_us * (FPB / 1000000)) / 8;
  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, timer_preset);
 
  // set up timer interrupt with a priority of 2 and sub-priority of 0
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

  // enable multi-vector interrupts
  INTEnableSystemMultiVectoredInt();
  
  snprintf (strBuf, bufLen, "Hello...\n");
  SendDataBuffer (strBuf, strlen(strBuf));

  // configure and enable the ADC

  CloseADC10();	// ensure the ADC is off before setting the configuration
  
  /*
      Previous version...
  //          set pin RA0 / AN0 as analog input
  // AD1PCFG = 0x0000fffe;
  // ANSELAbits.ANSA0 = 0;
  //          CH0NB negative input select for MUX B is Vref-
  //          CH0SB positive input for mux B channel 0 is AN0
  //          CH0NA negative input select for MUX A is Vref-
  //          CH0SA positive input for mux A channel 0 is AN0
  SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | POT_ADC_ITM
        | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN11);

  // 				Turn module on | ouput in integer | conversion begins with clock | disable autosample
  AD1CON1 = ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF;
  // 				ADC ref internal    | disable offset test    | disable scan mode | 1 samples / interrupt | use single buffer | use MUX A only
  AD1CON2 = ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF;
  //				  use ADC internal clock | set sample time
  // could also try ADC_CONV_CLK_PB which is the 125ns peripheral clock
  AD1CON3 = ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15;
  //          select pin AN0 for input scan
  // AD1CSSLbits.CSSL0 = 1;
  */
  
  /*
    Notes from ref chapter
      must set the TRIS bits for these channels (1 (desired) by default)
      AD1CON1
        want form 32-bit integer (0b100)
        we set the SAMP bit to begin sampling --no--
          the ASAM bit will begin autosampling
          combine this with CLRASAM which will clear ASAM after one set of samples
          conversion should be auto after that
        autoconvert when sampling has concluded (SSRC = 0b111)
        Tad min is 65ns, which is 2.6 PBclock cycles
        Tad min is 83.33ns, which is 3.3 PBclock cycles
      AD1CON2
        CSCNA on enables scanning of inputs according to AD1CSSL
        maybe BUFM 2 8-word buffers rather than 1 16-word buffer
        ALTS alternates between mux A and mux B
      AD1CON3
        ADRC use peripheral bus clock ADC_CONV_CLK_PB
        TAD: use ADC clock divisor of 4 to give TAD of 100ns to exceed min of 83.33
        Sample time (SAMC): use 2 TADs to provide 200ns sampling to exceed min of 132
              
        New strategy: set the bit to power the necessary bank, wait 200 us,
        and then kick off a sample.
        
      AD1CHS
        CH0SA and CH0SB not used when scanning channels; CH0NA and CH0NB *are* used
      AD1PCFG
        zero bits to configure channels as analog; set to zero on reset
      AD1CSSL
        one bits to select channel for scan
        
      Will want to scan all channels on one bank, then interrupt to switch banks
      Protect user code by writing valid stuff during interrupt to user area
      
      
      
      settling time after switching is say 200 us (78us to 0.63 rise)
      
      
      
      
  */

  AD1CON1 =   ADC_FORMAT_INTG32           // output in integer
            | ADC_CLK_AUTO                // conversion begins with clock
            | ADC_AUTO_SAMPLING_OFF       // don't yet start autosample
          ;

  AD1CON2 =   ADC_VREF_AVDD_AVSS          // ADC ref internal
            | ADC_OFFSET_CAL_DISABLE      // disable offset test
            | ADC_SCAN_ON                 // enable scan mode (CSCNA)
            | ADC_SAMPLES_PER_INT_5 
            | ADC_ALT_BUF_OFF             // use single buffer
            | ADC_ALT_INPUT_OFF           // use MUX A only
          ;
 
  AD1CON3 =   ADC_SAMPLE_TIME_2           // use 2 TADs for sampling
            | ADC_CONV_CLK_PB             // use PBCLK
            | ADC_CONV_CLK_Tcy            // 4 PBCLK cycles, so TAD is 100ns
          ;

  // AD1CHS
  //          CH0NA negative input select for MUX A is Vref-
  //          ASAM will begin a sample sequence
  //          CLRASAM will stop autosampling after completing one sequence
  //          begin conversions automatically
  SetChanADC10 (ADC_CH0_NEG_SAMPLEA_NVREF);
  
  // AD1PCFG
  
  // these are in the order in which they will appear in BUF0
  AD1CSSL =   LS0_ADC_ITM 
            | LS1_ADC_ITM 
            | LS2_ADC_ITM 
            | LS3_ADC_ITM 
            | POT_ADC_ITM
          ;  
  
  EnableADC10();

  while(1) {

    // enableADC is set periodically (ADC_interval_ms) by an interrupt service routine
    if (enableADC) {
//      snprintf (strBuf, bufLen, "ADC...\n");
//      SendDataBuffer (strBuf, strlen(strBuf));
      
      //*
      
      // NOTE: the interrupt routine set AD1CON1bits.SAMP = 1;
      // use AN0, pin 2
      
      // wait for the conversions to complete
      // so there will be vaild data in ADC result registers
      while ( ! AD1CON1bits.DONE );
  
      for (i = 0; i < NSENSORSINBANK; i++) {
        cSensor[bank][i] = ReadADC10(i);
      }
      cPot = POT_ADC_VAL;

      // current setup has 5 sensors:
      //   2 in bank 0
      //   3 in bank 1
      if (enablePrint) {
        snprintf (strBuf, bufLen, "%8d %8d %8d %8d %8d %8d\n", 
          cSensor[0][0],
          cSensor[0][1],
          cSensor[1][0],
          cSensor[1][1],
          cSensor[1][2],
          cPot);
        SendDataBuffer (strBuf, strlen(strBuf));
        enablePrint = 0;
      }
 
      //*/

      AD1CON1bits.DONE = 0;
      enableADC = 0;
      bank = 1 - bank;
      // power up the new bank
      if (bank == 0) {
        PWR_BANK1 = 0;
        PWR_BANK1_TRIS = 1;
        PWR_BANK0 = 1;
        PWR_BANK0_TRIS = 0;
      } else {
        PWR_BANK0 = 0;
        PWR_BANK0_TRIS = 1;
        PWR_BANK1 = 1;
        PWR_BANK1_TRIS = 0;
      }

      // time for new bank to stabilize is controlled by sampling interval
    }  // ADC enabled
  }  // infinite loop

  return -1;
}

void __ISR(_TIMER_1_VECTOR, ipl2) _Timer1Handler(void)
{
  // this interrupt should fire every 1 ms
  static int blinkRemainingUs = 0;
  static int adcRemainingUs = 0;
  static int printRemainingUs = print_interval_ms;

  blinkRemainingUs -= clock_interrupt_period_us;
  if (blinkRemainingUs <= 0) {
    LED = 1 - LED;
    blinkRemainingUs = LED_blink_period_ms * 1000;
  }
  adcRemainingUs -= clock_interrupt_period_us;
  if (adcRemainingUs <= 0) {
    enableADC = 1;
    AD1CON1bits.ASAM = 1;
    adcRemainingUs = ADC_interval_ms * 1000;
  }
  
  printRemainingUs -= clock_interrupt_period_us;
  if (printRemainingUs <= 0) {
    enablePrint = 1;
    printRemainingUs = print_interval_ms * 1000;
  }

  mT1ClearIntFlag(); // clear the interrupt flag
}