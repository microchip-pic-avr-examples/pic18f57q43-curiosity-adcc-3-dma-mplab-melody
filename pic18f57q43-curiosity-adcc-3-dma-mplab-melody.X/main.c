 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
 * 
 * @details This project demonstrates context switching for the ADCC
  * (ADC with Computation) peripheral. The switch is automated
  * using the DMA subsystem. The general flow is
  * 1. set up the system
  * 2. use DMA to configure the ADC and initialize the math context
  * 3. wait for the ADC conversion done, triggered by TMR0
  * 4. use DMA to save the updated ADC context and configuration
  * 5. install a new configuration
  * 6. print the resulting values from the saved context
  * 7. repeat indefinitely from step 3
  * 
  * The project configuration is managed by Melody, and requires
  * both TMR0 (periodic free running) and ADC (right-justified 
  * results, triggered by TMR0). All other parameters may be 
  * configured as desired.
*/

/*
� [2021] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"
#include "adcContext.h"
#include "Serout.h"
#include <assert.h>

/**
 * Conversion parameters used for individual channel set-up.
 * If this array is tagged as const, DMA3 must have DMA3CON1.SMR=Flash.
 * Parameters not specified will be zero. */
static adConfig_t adConfig[] = { // note that ADPCH is specified in octal
    { .cADPCH = 000, .cADACQ = 0x64, .cADCON0 = 0x84, .cADCON2 = 0x04, },
    { .cADPCH = 001, .cADACQ = 0x64, .cADCON0 = 0x84, .cADCON2 = 0x04, },
    { .cADPCH = 002, .cADACQ = 0x64, .cADCON0 = 0x84, .cADCON2 = 0x04, },
};
#define NUM_ADCONFIG    (sizeof(adConfig)/sizeof(adConfig[0]))

/**
 * Computation context cache for each of the conversion channels.
 * DMA1 and DMA2 copy into and out of this array. */
static volatile adContext_t adContext[NUM_ADCONFIG] = { {0} };

/*
            DMA functions
 */
#define VIEW_DMA(n) do{DMASELECT=((n)-1);}while(0)
#define UINT24_FROM_PTR(p)    ((uint24_t)(uintptr_t)(p))
#define UINT16_FROM_PTR(p)    ((uint16_t)(uintptr_t)(p))

/** 
 * Initialize the DMA control registers to provide the 3 different 
 * transfer operations. Because SSZ != DSZ, each trigger will transfer the 
 * smaller of the two, and the other will hold partially fulfilled.
 * The order of operations is controlled by the priority settings,
 * so these will run in order DMA1, DMA2, DMA3. */
void DMAforADC_Initialize(void)
{
	/* Set up DMA1 to copy the ADC control SFRs into the context cache. */
    VIEW_DMA(1); // Select DMA1
    DMAnCON0  = 0x00; // all configuration must take place with DMAEN=OFF
    DMAnSSA   = UINT24_FROM_PTR(&ADLTHL); // copy from hardware SFRs
    DMAnSSZ   = sizeof(adContext[0]);     // source size is just the necessary SFRs
    DMAnDSA   = UINT16_FROM_PTR(adContext); // first cache location
    DMAnDSZ   = sizeof(adContext);          // using the full cache
    DMAnSIRQ  = 0x0A; // transfer trigger is AD conversion complete
    DMAnAIRQ  = 0x00; // abort trigger is nil
    DMAnCON1  = 0x42; // GPR is source, increment DPTR and SPTR
    DMAnCON0  = 0xC0; // enable, hardware trigger, no abort

	/* Set up DMA2 to copy the context cache into the ADC control SFRs. */
    VIEW_DMA(2); // Select DMA2
    DMAnCON0  = 0x00; // all configuration must take place with DMAEN=OFF
    DMAnSSA   = UINT24_FROM_PTR(adContext); // begin copying from start of cache
    DMAnSSZ   = sizeof(adContext);          // using the full cache
    DMAnDSA   = UINT16_FROM_PTR(&ADLTHL); // copy to hardware SFRs
    DMAnDSZ   = sizeof(adContext[0]);     // just the necessary SFRs
    DMAnSIRQ  = 0x0A; // transfer trigger is AD conversion complete
    DMAnAIRQ  = 0x00; // abort trigger is nil
    DMAnCON1  = 0x42; // GPR is source, increment DPTR and SPTR
    DMAnCON0  = 0xC0; // enable, hardware trigger, no abort

	/* Set up DMA3 to write a new configuration to the ADC SFRs. */
    VIEW_DMA(3); // Select DMA3
    DMAnCON0  = 0x00; // all configuration must take place with DMAEN=OFF
    DMAnSSA   = UINT24_FROM_PTR(adConfig); // begin copying from start of cache
    DMAnSSZ   = sizeof(adConfig);          // using the full cache
    DMAnDSA   = UINT16_FROM_PTR(&ADPCH); // copy to hardware SFRs
    DMAnDSZ   = sizeof(adConfig[0]);     // just the necessary SFRs
    DMAnSIRQ  = 0x0A; // transfer trigger is AD conversion complete
    DMAnAIRQ  = 0x00; // abort trigger is nil
    DMAnCON1  = 0x42; // GPR is source, increment DPTR and SPTR
    DMAnCON0  = 0xC0; // enable, hardware trigger, no abort

    /* Unlock the arbiter to set the DMA priorities. Different values are
     * used to make sure that the transfers happen in proper order
     * since they are all triggered from the ADC-complete interrupt:
     * 1. copy the (now updated) math context to cache
     * 2. install the context for the next sample, and
     * 3. load the configuration for the next sample, including
          the channel number, acquisition timing, and ADCC mode. */
	asm ("BANKSEL PRLOCK");
    asm ("MOVLW 0x55");
    asm ("MOVWF PRLOCK");
    asm ("MOVLW 0xAA");
    asm ("MOVWF PRLOCK");
    asm ("BSF PRLOCK, 0");
	DMA1PR = 0; //set DMA1 to highest priority (context read)
	DMA2PR = 1; //set DMA2 to next highest priority (context write)
	DMA3PR = 2; //set DMA3 to just above interrupt (config write)
}

/*
    Main application
*/
void main(void)
{
    SYSTEM_Initialize();    // Melody peripheral configuration and startup

    T0CON0bits.EN = 0;      // pause the timer during set-up
	DMAforADC_Initialize(); // Setup DMA to perform ADC context switch
    
    /* Two of the DMA are used to load ADC registers (the 3rd reads
     * registers). Trigger register loading now to initialize the ADC
     * for the first measurement. */
    VIEW_DMA(2);          // select DMA2 (for adcontext)
	DMAnCON0bits.DGO = 1; // trigger DMA2 to load context for channel 0
    VIEW_DMA(3);          // select DMA3 (for adconfig)
	DMAnCON0bits.DGO = 1; // trigger DMA3 to load configuration for channel 0

    T0CON0bits.EN = 1;      // start the timer to trigger the ADC

    while(1) // Main while-one loop
    {
        /* This section should be a separate task but is included here
         * to keep the example simple. */
        /* Enter after DMA1 has completed the data fetch. */
        if (0 != PIR2bits.DMA1SCNTIF)
        {
            /* Track the ADC progress using a counter. 
             * Configuration #0 was loaded during initialization. */
            static uint8_t dmaStage = 0;
            
            /* Clear the flag and capture the result. */
            PIR2bits.DMA1SCNTIF = 0;
            uint16_t adc_result = adContext[dmaStage].cADRES;

            /* Send the result */
            Serout(dmaStage, adc_result);

            /* Advance and validate the stage counter */
            dmaStage++;
            if (dmaStage >= NUM_ADCONFIG) dmaStage = 0;

#if 1
            /* Verify that the DMA pointers have moved correctly.
             * This is done for the sake of the demonstration, and
             * is not necessary in production code. */
            VIEW_DMA(1);
            __conditional_software_breakpoint(DMAnSPTR == UINT24_FROM_PTR(&ADLTHL));
            __conditional_software_breakpoint(DMAnDPTR == UINT16_FROM_PTR(&adContext[dmaStage]));

            uint8_t nextStage = dmaStage + 1;
            if (nextStage >= NUM_ADCONFIG) nextStage = 0;

            VIEW_DMA(2);
            __conditional_software_breakpoint(DMAnSPTR == UINT24_FROM_PTR(&adContext[nextStage]));
            __conditional_software_breakpoint(DMAnDPTR == UINT16_FROM_PTR(&ADLTHL));
            VIEW_DMA(3);
            __conditional_software_breakpoint(DMAnSPTR == UINT24_FROM_PTR(&adConfig[nextStage]));
            __conditional_software_breakpoint(DMAnDPTR == UINT16_FROM_PTR(&ADPCH));
            NOP();
#endif
        }
    }
}
