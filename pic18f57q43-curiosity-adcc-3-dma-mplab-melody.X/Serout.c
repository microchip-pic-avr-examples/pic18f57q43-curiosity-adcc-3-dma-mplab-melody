/*
 * @file serout.c
 * @brief Simple serial output.
 * @version 1.0.0
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

/*
                         Constant Definitions
 */
#define LF	0x0A
#define CR	0x0D

static void UART_Write(char c)
{
    while (UART2_IsTxReady() == 0);  // Wait for UART to be idle
	UART2_Write(c);
}

// Send the end of line CR/LF
void EOL(void)
{
	UART_Write(CR);   // send CR
	UART_Write(LF);   // send LF
}

// Send comma and space between values
void CDT(void)
{
	UART_Write(','); // send ,
	UART_Write(' '); // send space
}

void Serout(uint8_t index, uint16_t valu)
{
	uint8_t	 cntr;
	uint16_t temp;

    /* A little bit of formatting for the serial output
     * then send the result. */
    if (0 == index) EOL(); // new line
    else CDT(); // comma

	UART_Write('0');      // print "0x" at start of line
	UART_Write('x');
	for	(cntr=0; cntr<4; cntr++)           // print 4 hex digits of 16-bit value
	{
		temp = (valu & 0xF000);            // Convert most significant nibble to ASCII
		temp = temp >> 12;
		temp += '0';
		if (temp > '9') temp += 7;
		UART_Write(temp & 0xFF);           // Send the nibble
		valu = valu << 4;
	}
}
