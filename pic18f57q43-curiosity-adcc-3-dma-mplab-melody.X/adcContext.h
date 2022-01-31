/*
 * @file adcContext.h
 * @brief In-memory model of the registers for ADCC.
 * @version 1.0.0
*/

/*
© [2021] Microchip Technology Inc. and its subsidiaries.

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

#ifndef ADCCONTEXT_H
#define	ADCCONTEXT_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <inttypes.h>
#define PACK    /* __pack */
    
    /**
     * Mirror the SFRs that configure the ADCC and ADC3 logic.
     * This data may be copied from or to the actual ADC hardware SFRs
     * in order to archive or restore a specific set-up.
     *  */
    typedef PACK struct adConfig_s
    {
        uint8_t  cADPCH;     /// ADC channel to convert
        uint8_t  cADNCH;     /// Used only with differential ADC
        uint16_t cADACQ;     /// Pre-conversion acquisition time
        uint8_t  cADCAP;     /// Additional capacitance selector
        uint16_t cADPRE;     /// Pre-acquisition precharge time
        uint8_t  cADCON0;    /// Control register 0
        uint8_t  cADCON1;    /// Control register 1
        uint8_t  cADCON2;    /// Control register 2
        uint8_t  cADCON3;    /// Control register 3
        uint8_t  cADSTAT;    /// Conversion status
#if 0
        /* Some applications may want to change these for each 
         * channel, but not for this example. */
        uint8_t  cADREF;     /// Conversion reference selector (N & P)
        uint8_t  cADACT;     /// Auto-conversion trigger selector
        uint8_t  cADCLK;     /// Clock selector
#endif
    } adConfig_t;

    /***
     * Mirror the SFRs that contain the computational context of the ADC.
     * This data may be copied from or to the actual ADC hardware SFRs
     * in order to archive or restore a specific data set.
     * */
    typedef PACK struct adContext_s
    {
        uint16_t cADLTH;    /// Lower threshold
        uint16_t cADUTH;    /// Upper threshold
        uint16_t cADERR;    /// Calculated error
        uint16_t cADSTPT;   /// Setpoint
        uint16_t cADFLTR;   /// Filtered results
        uint24_t cADACC;    /// Accumulator
        uint8_t  cADCNT;    /// Sample counter
        uint8_t  cADRPT;    /// Repetition count
        uint16_t cADPREV;   /// Previous measurement
        uint16_t cADRES;    /// Current measurement
#if 0
        /* Some applications may want to capture the channel#
         * with the ADRES value, but not for this example. */
        uint8_t  cADPCH;     /// ADC channel converted
#endif
    } adContext_t;
    
    /* This test verifies that the compiler has correctly constructed
     * the cache and config areas. The expected sizes correspond to
     * the definition of the ADC peripheral. */
    typedef uint8_t bad_adConfig[(12 == sizeof(adConfig_t)? 1 : -1)];
    typedef uint8_t bad_adContext[(19 == sizeof(adContext_t)? 1 : -1)];

#ifdef	__cplusplus
}
#endif

#endif	/* ADCCONTEXT_H */

