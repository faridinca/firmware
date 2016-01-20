#include "c8051F340.h"
//#include "as399x_config.h"
//#include "c8051F340_defs.h"  
#include "stdio.h"
#include "stdlib.h"

#include "platform.h"

#define GLOBALS_
#include "globals.h"

//code char BUILD_VERSION[] =	"Build: 01-27-2014";
code char BUILD_VERSION[] =	"Build: 02-24-2015";

#define PCA_BUFF_SZ     40

// '0' nominal is 14
#define PCA_0_LL        10
#define PCA_0_UL        18

// '1' nominal is 22
#define PCA_1_LL        19
#define PCA_1_UL        27

// bit gap nominal is   40
#define PCA_BIT_GAP_LL  30
#define PCA_BIT_GAP_UL  50

// reapt cycle nominal is 40000
#define PCA_REPEAT_CYCLE_LL 100
#define PCA_REPEAT_CYCLE_UL 60000

data u8 lsbPCABuff[PCA_BUFF_SZ];
data u8 msbPCABuff[PCA_BUFF_SZ];
data volatile u8 pcaBuffIndexHead;
data u8 pcaBuffIndexTail;

idata u16 pcaDataPrev;
idata u16 pcaData;
idata u16 pcaDiff;
bit pcaBit;
idata u8 tmpIndex;

extern xdata char tempBuff[];

void DumpPCAData(void);

void main(void)				
{
	/*
    u8 i;
    u16 pcaDataPrev = 0;
    u16 pcaData = 0;
    u16 pcaDiff = 0;
	*/

    PCA0MD &= ~0x40;                    // WDTE = 0, clear watchdog timer 
    EA = 0;                             // disable all interrupts

    InitSystem();
    
    while(1)
    {
        if(newTick == TRUE)
        {
            newTick = FALSE;
            ProcessCom();
            ToggleLED();    
        }
    }
}



/*
 * comment_3
 */


/*
*
*/
void InitSystem(void)
{
    
    InitOsc();
    InitPorts();           // Initialize crossbar and GPIO

    InitParam();
    InitTimers();
    InitUART();
    uartTxData(BUILD_VERSION);

    DisableTimer0IRQ;

    InitAMS3992Interface();
    
    //
    // Following delay is per AMS source code comments
    //
    // AS3992 needs this, test showed the border at about 3us
    //
    mdelay(1); 
                 
    // Enable global interrupts
    EA = 1; 

    StartASM3992(866900);
    
    P3MDOUT |= 0xC0;
    P3 &= 0x3F;    
#if ENABLE_SUBCARRIER_OUTPUT == 0
    MODULTION_PORT = 1;
#endif

    EnableTimer0IRQ;
	EA = 1;

#if ENABLE_SIMULATION
    SIMULATION_PORT = 1;
#endif
	
                        
}

/*
 * comment_4
 */


/*----------------------------------------------------------------------------- */
/* Delay */
/*----------------------------------------------------------------------------- */
/* */
/* Return Value - None */
/* Parameters - None */
/* */
/* Used for a small pause, approximately 80 us in Full Speed, */
/* and 1 ms when clock is configured for Low Speed */
/* */
/* ---------------------------------------------------------------------------- */
void Delay(void)
{

    int x;
    for(x = 0; x < 500; x++);
}


/**
* InitOsc()
* @brief Initialize& configure oscillator.
* This function initializes the system clock to use the internal 12/4 MHz
* oscillator as its clock source.  Also enables missing clock detector reset.
* @parameter None
* @return None
*/

void InitOsc(void)
{
    /*
   OSCICN = 0x81;                      // Configure internal oscillator for
                                       // its lowest frequency
   RSTSRC = 0x04;                      // Enable missing clock detector
   */
    
    //
    // Initialize oscillator
    //
    OSCICN |= 0x03;                     // Configure internal oscillator for
                                        // its maximum frequency and enable
                                        // missing clock detector

    CLKMUL  = 0x00;                     // Select internal oscillator as
                                        // input to clock multiplier

    CLKMUL |= 0x80;                     // Enable clock multiplier 
    Delay();                            // Delay for clock multiplier to begin
    CLKMUL |= 0xC0;                     // Initialize the clock multiplier
    Delay();                            // Delay for clock multiplier to begin

    while (!(CLKMUL & 0x20));           // Wait for multiplier to lock

    FLSCL   = 0x90;                     // Set flash waitstates for freqs > 25 MHz
    CLKSEL  = SYS_4X | USB_4X_CLOCK;    // Set system clock to 48MHz
    
}

/**
 * InitPort()
// @brief This function configures the crossbar and GPIO ports.
*  @parameter None
*  @return None
*/
//
// P0.0   digital   open-drain    /INT0
// P0.4   digital   push-pull     UART TX
// P0.5   digital   open-drain    UART RX
// P2.0   digital   open-drain    SW push button switch
// P2.2   digital   push-pull     LED (and CEX0 depending on Blink_State)
// P2.5   analog                  Potentiometer (ADC input)
//
//-----------------------------------------------------------------------------
void InitPorts (void)
{
   //P2MDIN   &= ~0x20;                 // P2.5 is analog, rest of P1 digital
   P0MDOUT |= 0xD0;                     // P0.7, P0.6 & P0.4 are push-pull
   //P2MDOUT   = 0x04;                  // P2.2 is push-pull

   //P0SKIP = 0xCF;                       // Skip all of P0 except for UART0 pins
   
   
   //P1SKIP    = 0xFF;                  // Skip all of P1
   //P2SKIP    = 0x03;                  // Skip P2.0 and P2.1

   P0 |= 0x21;                          // Set port latch for pins RX and /INT0
                                        // to configure as inputs

   XBR0 |= 0x01;                       // UART0 TX0, RX0 routed to Port pins P0.4 and P0.5
   XBR1 |= 0x40;                        // T0 routed to Port pin, Enable Crossbar; pull-ups enabled

    P4MDOUT = 0x00;
    P4 = (~P4MDOUT);


}

/**
 * InitAMS3992Interface()
// @briefThis function initializes the in/out ports of the controller for communication with AS399x via SPI.
*  @parameter None
*  @return None
*/

void InitAMS3992Interface(void)
{
    // P0 Port: IO4,7, LED and CLK output; IRQ, IO6, uart rx, clksys input
    P0MDOUT = 0x55; 
    P0 = (~P0MDOUT) | 0x40;
    
    // P1 Port: all outputs
    P1MDOUT = 0xFF;
    // disable tune caps and switch on antenna P1
    P1 = (~P1MDOUT) | 0x43;    
    
    // P2 Port: all outputs except IO5
    P2MDOUT = 0xE7;
    // enable PA_LDO and VDD_IO, set IO5 as input             
    P2 = (~P2MDOUT) | 0x16;     
    
    // IO1=1, IO0=0, enable of AS399x set to high later (StartASM3992)
    P3MDOUT = 0x0B;
    P3 = (~P3MDOUT) | 0x01;    
    
    P4MDOUT = 0x00;
    P4 = (~P4MDOUT);

   //  MSTEN=1, CPHA=1, CKPOL=0
    SPI0CFG = 0x60;
#if (CLK == 48000000)       // reg = f_sys/(2f) - 1
    SPI0CKR = 48/(2*2)-1;   // 2 MHz
#elif (CLK == 24000000)
    SPI0CKR = 24/(2*2)-1;   // 2 MHz
#elif (CLK == 12000000)
    SPI0CKR = 12/(2*2)-1;   // 2 MHz
#endif


    // 3-wire, enable SPI
    SPI0CN  = 1;
     
    //
    // enable external interrupt
    //

    // put Portpin in tristate
    IRQPIN = 1;
    
    // interrupt active high and use P0.3 for Interrupt Input Pin          
    IT01CF = 0x0B;
    // rising edge triggered       
    IT0 = 1;

    // Enable external interrupts             
    EX0 = 1; 
    //IE |= 0x01;
    

    // Enable all interrupts            
    EA = 1;

    // enable SPI interface
    XBR0 |= 0x2;
    // enable the crossbar, disable weak pull ups  
    XBR1 |= 0xC0; 

    // SPI cs_ 
    IO4(1); 
   

    EN(LOW); 
    udelay(100);

    // now we should be in SPI mode
    EN(HIGH);
    
    
}


/**
 * InitPCA()
// @briefThis function initializes the PCA capture module interface.
*  @parameter None
*  @return None
*/
void InitPCA(void)
{
	u8 i;

    for(i = 0; i < PCA_BUFF_SZ; i++)
    {
        lsbPCABuff[i] = msbPCABuff[i] = 0;
    }
    pcaBuffIndexHead = 0;
    pcaBuffIndexTail = 0;

    sensorSignData = 0;
    sensorData = 0;
    sensorBitIndex = 0;
}


/**
* InitParam()
* @brief Initialize the global varaiables and parameters
* @parameter None
* @return None
*/

void InitParam(void)
{
    newTick = FALSE;
    tickCounter = 0;
    ledTickCounter = 0;

    accessRegisters = 1;

    InitPCA();

}


/**
* ToggleLED()
* @brief Turning on/off the LEDs. An inidcator that the board is alive
* @parameter None
* @return None
*/

void ToggleLED(void)
{
    ++ledTickCounter;

    if(ledTickCounter < ONE_SECOND)
        LED(0);
    else if(ledTickCounter < TWO_SECONDS)
        LED(1);
    else
    {
        ledTickCounter = 1;
        LED(0);
    }
}



//-----------------------------------------------------------------------------
// PCA0_ISR
//-----------------------------------------------------------------------------
//
//
// This is the ISR for the PCA.  It handles the case when a capture occurs on
// channel 1, and updates the variables to reflect the new capture information.
//
// Return Value : None
// Parameters   : None
//-----------------------------------------------------------------------------


void PCA0_ISR (void) interrupt 11 using 2
{
    //P3_6 = 0;

    if(CCF1)                           // If Module 1 caused the interrupt
    {

        {
            // Store most recent capture value
            msbPCABuff[pcaBuffIndexHead] = PCA0CPH1;
            lsbPCABuff[pcaBuffIndexHead] = PCA0CPL1;

            if(++pcaBuffIndexHead >= PCA_BUFF_SZ)
                pcaBuffIndexHead = 0;
        }

        CCF1 = 0;                       // Clear module 1 interrupt flag.
   }
   else                                 // Interrupt was caused by other bits.
   {
        //PCA0CN &= ~0xBF;                // Clear other interrupt flags for PCA
        PCA0CN &= 0x42;                // Clear other interrupt flags for PCA
   }
   
   //P3_6 = 1;
}


void DumpPCAData(void)
{
    u8 i;
    pcaDataPrev = 0;
    pcaData = 0;
    pcaDiff = 0;

    for(i = 0; i < PCA_BUFF_SZ; i++)
    {
        pcaData = (u16)msbPCABuff[i];
        pcaData <<= 8;
        pcaData &= 0xFF00;
        pcaData |= (u16)lsbPCABuff[i];
        
        if(i)
        {
            pcaDataPrev = (u16)msbPCABuff[i-1];
            pcaDataPrev <<= 8;
            pcaDataPrev &= 0xFF00;
            pcaDataPrev |= (u16)lsbPCABuff[i-1];

            if(pcaData >= pcaDataPrev)
                pcaDiff = (pcaData - pcaDataPrev);
            else
            {
                pcaDiff = pcaData;
                pcaDiff += (0xFFFF - pcaDataPrev);
            }    
        }

		sprintf(tempBuff, "%2d --- 0x%04X %u", (int)i, pcaData, pcaDiff);
        uartTxData(tempBuff);
        WaitTxCompletion();
        
        NOP();
    }

}

/**
* DecodeRFData()
* @brief Decoding the sensor data
* @parameter None
* @return None
*/

void DecodeRFData(void)
{
    u8 diff;


    while(pcaBuffIndexHead != pcaBuffIndexTail)
    {
        if(pcaBuffIndexHead >= pcaBuffIndexTail)
            diff = (pcaBuffIndexHead - pcaBuffIndexTail);
        else
        {
            diff = (PCA_BUFF_SZ - pcaBuffIndexTail);
            diff += pcaBuffIndexHead;
        }
        
        if(diff < 2)
            return;


        tmpIndex = pcaBuffIndexTail;

        if(++pcaBuffIndexTail >= PCA_BUFF_SZ)
            pcaBuffIndexTail = 0; 

        if(pcaBuffIndexHead > 1)
        {
            pcaData = (u16)msbPCABuff[pcaBuffIndexTail];
            pcaData <<= 8;
            pcaData &= 0xFF00;
            pcaData |= (u16)lsbPCABuff[pcaBuffIndexTail];

            pcaDataPrev = (u16)msbPCABuff[tmpIndex];
            pcaDataPrev <<= 8;
            pcaDataPrev &= 0xFF00;
            pcaDataPrev |= (u16)lsbPCABuff[tmpIndex];

            if(pcaData >= pcaDataPrev)
                pcaDiff = (pcaData - pcaDataPrev);
            else
            {
                pcaDiff = pcaData;
                pcaDiff += (0xFFFF - pcaDataPrev);
            }
            
            if( (pcaDiff >= PCA_0_LL)  && (pcaDiff <= PCA_0_UL))
            {
                pcaBit = 0;
            }
            else if( (pcaDiff >= PCA_1_LL)  && (pcaDiff <= PCA_1_UL))
            {
                pcaBit = 1;
            }
            else if(pcaDiff >= PCA_REPEAT_CYCLE_LL)
            {
                sensorBitIndex = 0;
                sensorData = 0;
                continue;
            }
            else
			{
				NOP();
                continue;
			}

            if(sensorBitIndex == 0)
            {
                sensorSignData = (u8)pcaBit;
                sensorBitIndex = 1;
            }
            else
            {
                sensorData >>= 1;
                sensorData &= 0x7F;
				if(pcaBit)
                	sensorData |= 0x80;
                ++sensorBitIndex; 
            }    

            if(sensorBitIndex >= 9)
            {
                uartTxBinaryData(sensorSignData, sensorData);
                sensorBitIndex = 0;
                sensorData = 0;
            }

            NOP();
        }
        else
        {
           pcaBuffIndexTail = tmpIndex;
        }  
    }
    
    
}


