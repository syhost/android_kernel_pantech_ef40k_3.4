// filename: main.c
#include <linux/issp_revision.h>
#ifdef PROJECT_REV_304
/* Copyright 2006-2007, Cypress Semiconductor Corporation.
//
// This software is owned by Cypress Semiconductor Corporation (Cypress)
// and is protected by and subject to worldwide patent protection (United
// States and foreign), United States copyright laws and international
// treaty provisions. Cypress hereby grants to licensee a personal,
// non-exclusive, non-transferable license to copy, use, modify, create
// derivative works of, and compile the Cypress Source Code and derivative
// works for the sole purpose of creating custom software in support of
// licensee product to be used only in conjunction with a Cypress integrated
// circuit as specified in the applicable agreement. Any reproduction,
// modification, translation, compilation, or representation of this
// software except as specified above is prohibited without the express
// written permission of Cypress.
//
// Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,
// WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Cypress reserves the right to make changes without further notice to the
// materials described herein. Cypress does not assume any liability arising
// out of the application or use of any product or circuit described herein.
// Cypress does not authorize its products for use as critical components in
// life-support systems where a malfunction or failure may reasonably be
// expected to result in significant injury to the user. The inclusion of
// Cypress� product in a life-support systems application implies that the
// manufacturer assumes all risk of such use and in doing so indemnifies
// Cypress against all charges.
//
// Use may be limited by and subject to the applicable Cypress software
// license agreement.
//
//---------------------------------------------------------------------------*/

/* ############################################################################
   ###################  CRITICAL PROJECT CONSTRAINTS   ########################
   ############################################################################

   ISSP programming can only occur within a temperature range of 5C to 50C.
   - This project is written without temperature compensation and using
     programming pulse-widths that match those used by programmers such as the
     Mini-Prog and the ISSP Programmer.
     This means that the die temperature of the PSoC device cannot be outside
     of the above temperature range.
     If a wider temperature range is required, contact your Cypress Semi-
     conductor FAE or sales person for assistance.

   The project can be configured to program devices at 5V or at 3.3V.
   - Initialization of the device is different for different voltages. The
     initialization is hardcoded and can only be set for one voltage range.
     The supported voltages ranges are 3.3V (3.0V to 3.6V) and 5V (4.75V to
     5.25V). See the device datasheet for more details. If varying voltage
     ranges must be supported, contact your Cypress Semiconductor FAE or sales
     person for assistance.
   - ISSP programming for the 2.7V range (2.7V to 3.0V) is not supported.

   This program does not support programming all PSoC Devices
   - It does not support obsoleted PSoC devices. A list of devices that are
     not supported is shown here:
         CY8C22x13 - not supported
         CY8C24x23 - not supported (CY8C24x23A is supported)
         CY8C25x43 - not supported
         CY8C26x43 - not supported
   - It does not suport devices that have not been released for sale at the
     time this version was created. If you need to ISSP program a newly released
     device, please contact Cypress Semiconductor Applications, your FAE or
     sales person for assistance.
     The CY8C20x23 devices are not supported at the time of this release.

   ############################################################################
   ##########################################################################*/


/* (((((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))
 PSoC In-System Serial Programming (ISSP) Template
 This PSoC Project is designed to be used as a template for designs that
 require PSoC ISSP Functions.

 This project is based on the AN2026 series of Application Notes. That app
 note should be referenced before any modifications to this project are made.

 The subroutines and files were created in such a way as to allow easy cut &
 paste as needed. There are no customer-specific functions in this project.
 This demo of the code utilizes a PSoC as the Host.

 Some of the subroutines could be merged, or otherwise reduced, but they have
 been written as independently as possible so that the specific steps involved
 within each function can easily be seen. By merging things, some code-space
 savings could be realized.

 As is, and with all features enabled, the project consumes approximately 3500
 bytes of code space, and 19-Bytes of RAM (not including stack usage). The
 Block-Verify requires a 64-Byte buffer for read-back verification. This same
 buffer could be used to hold the (actual) incoming program data.

 Please refer to the compiler-directives file "directives.h" to see the various
 features.

 The pin used in this project are assigned as shown below. The HOST pins are
 arbitrary and any 3 pins could be used (the masks used to control the pins
 must be changed). The TARGET pins cannot be changed, these are fixed function
 pins on the PSoC.
 The PWR pin is used to provide power to the target device if power cycle
 programming mode is used. The compiler directive RESET_MODE in ISSP_directives.h
 is used to select the programming mode. This pin could control the enable on
 a voltage regulator, or could control the gate of a FET that is used to turn
 the power to the PSoC on.
 The TP pin is a Test Point pin that can be used signal from the host processor
 that the program has completed certain tasks. Predefined test points are
 included that can be used to observe the timing for bulk erasing, block
 programming and security programming.

      SIGNAL  HOST  TARGET
      ---------------------
      SDATA   P1.0   P1.0
      SCLK    P1.1   P1.1
      XRES    P2.0   XRES
      PWR     P2.1   Vdd
      TP      P0.7   n/a

 For test & demonstration, this project generates the program data internally.
 It does not take-in the data from an external source such as I2C, UART, SPI,
 etc. However, the program was written in such a way to be portable into such
 designs. The spirit of this project was to keep it stripped to the minimum
 functions required to do the ISSP functions only, thereby making a portable
 framework for integration with other projects.

 The high-level functions have been written in C in order to be portable to
 other processors. The low-level functions that are processor dependent, such
 as toggling pins and implementing specific delays, are all found in the file
 ISSP_Drive_Routines.c. These functions must be converted to equivalent
 functions for the HOST processor.  Care must be taken to meet the timing
 requirements when converting to a new processor. ISSP timing information can
 be found in Application Note AN2026.  All of the sections of this program
 that need to be modified for the host processor have "PROCESSOR_SPECIFIC" in
 the comments. By performing a "Find in files" using "PROCESSOR_SPECIFIC" these
 sections can easily be identified.

 The variables in this project use Hungarian notation. Hungarian prepends a
 lower case letter to each variable that identifies the variable type. The
 prefixes used in this program are defined below:
  b = byte length variable, signed char and unsigned char
  i = 2-byte length variable, signed int and unsigned int
  f = byte length variable used as a flag (TRUE = 0, FALSE != 0)
  ab = an array of byte length variables


 After this program has been ported to the desired host processor the timing
 of the signals must be confirmed.  The maximum SCLK frequency must be checked
 as well as the timing of the bulk erase, block write and security write
 pulses.

 The maximum SCLK frequency for the target device can be found in the device
 datasheet under AC Programming Specifications with a Symbol of "Fsclk".
 An oscilloscope should be used to make sure that no half-cycles (the high
 time or the low time) are shorter than the half-period of the maximum
 freqency. In other words, if the maximum SCLK frequency is 8MHz, there can be
 no high or low pulses shorter than 1/(2*8MHz), or 62.5 nsec.

 The test point (TP) functions, enabled by the define USE_TP, provide an output
 from the host processor that brackets the timing of the internal bulk erase,
 block write and security write programming pulses. An oscilloscope, along with
 break points in the PSoC ICE Debugger should be used to verify the timing of
 the programming.  The Application Note, "Host-Sourced Serial Programming"
 explains how to do these measurements and should be consulted for the expected
 timing of the erase and program pulses.
*/

/* ------------------------------------------------------------------------- */
/* Header------------------------------------------------------------------ -*/
/* ------------------------------------------------------------------------- */	

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/issp_extern.h>
#include <linux/issp_directives.h>
#include <linux/issp_defs.h>
#include <linux/issp_errors.h>

/* ------------------------------------------------------------------------- */
/* Define	------------------------------------------------------------------ -*/
/* ------------------------------------------------------------------------- */				
#define ISSP_DEBUG_ENABLE

#ifdef ISSP_DEBUG_ENABLE
#define CYTTSP_DBG(fmt, args...)  printk("[Cyttsp][%s, %d]" fmt, __FUNCTION__, __LINE__, ##args)
#else
#define CYTTSP_DBG(fmt, args...)
#endif

/* ------------------------------------------------------------------------- */
/* Value	------------------------------------------------------------------ -*/
/* ------------------------------------------------------------------------- */
unsigned char abSecurityDataPtr;
unsigned char bBankCounter;
unsigned int  iBlockCounter;
unsigned int  iChecksumData;
unsigned int  iChecksumTarget;
int binary_index;


/* ========================================================================= */
// ErrorTrap()
// Return is not valid from main for PSOC, so this ErrorTrap routine is used.
// For some systems returning an error code will work best. For those, the
// calls to ErrorTrap() should be replaced with a return(bErrorNumber). For
// other systems another method of reporting an error could be added to this
// function -- such as reporting over a communcations port.
/* ========================================================================= */
void ErrorTrap(unsigned char bErrorNumber)
{
    #ifndef RESET_MODE
        // Set all pins to highZ to avoid back powering the PSoC through the GPIO
        // protection diodes.
        SetSCLKHiZ();
        SetSDATAHiZ();
        // If Power Cycle programming, turn off the target
//        RemoveTargetVDD();
    #endif

	CYTTSP_DBG(">>>> ISSP Error : %d <<<<\n", bErrorNumber);
	CYTTSP_DBG(">>>> Plese try again\n");
    //while (1);
    // return(bErrorNumbers);
}

/* ========================================================================= */
/* MAIN LOOP                                                                 */
/* Based on the diagram in the AN2026                                        */
/* ========================================================================= */

int touch_update_main(unsigned char bBlack)
{

    int retval = PASS, nRcvError = 0;
    // -- This example section of commands show the high-level calls to -------
    // -- perform Target Initialization, SilcionID Test, Bulk-Erase, Target ---
    // -- RAM Load, FLASH-Block Program, and Target Checksum Verification. ----

    // >>>> ISSP Programming Starts Here <<<<
    CYTTSP_DBG(">>>> ISSP Programming Starts <<<<\n");
	
    // Acquire the device through reset or power cycle
    #ifdef RESET_MODE
        // Initialize the Host & Target for ISSP operations
        if (fIsError = fXRESInitializeTargetForISSP())
        {
            retval = fIsError;
            ErrorTrap(fIsError);
	     return fIsError;
        }
    #else
        // Initialize the Host & Target for ISSP operations
        fIsError = fPowerCycleInitializeTargetForISSP();
        if (fIsError)
        {
            CYTTSP_DBG( "error - Initialize target for ISSP : %d\n", fIsError);
            retval = fIsError;
            ErrorTrap(fIsError);
	     return fIsError;
        }
    #endif

    CYTTSP_DBG(">>>> Run the SiliconID Verification, and proceed according to result <<<<\n");

    // Run the SiliconID Verification, and proceed according to result.
    fIsError = fVerifySiliconID();
    if (fIsError)
    {
	nRcvError = 1;
#if 1
        CYTTSP_DBG( "error - siliconID verification : %d\n", fIsError);
        retval = fIsError;
        ErrorTrap(fIsError);
        return fIsError;
#endif
    }

    CYTTSP_DBG(">>>> Bulk-Erase the Device <<<<\n");

    #if 1
        // Bulk-Erase the Device.
        fIsError = fEraseTarget();
        if (fIsError)
        {
            CYTTSP_DBG("error - bulk-erase : %d\n", fIsError);
            retval = fIsError;
            ErrorTrap(fIsError);
	     return fIsError;
        }

    #endif

    CYTTSP_DBG(">>>> program flash block <<<<\n");

    #if 1   // program flash block
        //==============================================================//
        // Program Flash blocks with predetermined data. In the final application
        // this data should come from the HEX output of PSoC Designer.
	 binary_index = 0;
	 iChecksumData = 0;     // Calculte the device checksum as you go
        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {
        	msleep(1);
            fIsError = fReadWriteSetup();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadWriteSetup : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;
            }

            LoadProgramData(bBankCounter, (unsigned char)iBlockCounter, bBlack);
            iChecksumData += iLoadTarget();

            fIsError = fProgramTargetBlock(bBankCounter,(unsigned char)iBlockCounter);
            if (fIsError)
            {
                CYTTSP_DBG("error - fProgramTargetBlock : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;				
            }

//			printk("Write Block: %d\n", iBlockCounter);

            fIsError = fReadStatus();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadStatus : %d\n", fIsError);
                retval = fIsError;				
                ErrorTrap(fIsError);
	         return fIsError;				
            }

        }

    #endif

//#if 0
    if(nRcvError == 1)  // verify
    {
    	CYTTSP_DBG(">>>> verify <<<<\n");
        //=======================================================//
        //PTJ: Doing Verify
        //PTJ: this code isnt needed in the program flow because we use PROGRAM-AND-VERIFY (ProgramAndVerify SROM Func)
        //PTJ: which has Verify built into it.
        // Verify included for completeness in case host desires to do a stand-alone verify at a later date.
	 binary_index = 0;
        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {
        	LoadProgramData(bBankCounter, (unsigned char) iBlockCounter ,bBlack);

            fIsError = fReadWriteSetup();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadWriteSetup : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;				
            }

            fIsError = fVerifySetup(bBankCounter,(unsigned char)iBlockCounter);
            if (fIsError)
            {
                CYTTSP_DBG("error - fVerifySetup : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;				
            }

            fIsError = fReadStatus();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadStatus : %d\n", fIsError);
                retval = fIsError;				
                ErrorTrap(fIsError);
	         return fIsError;				
            }

            fIsError = fReadWriteSetup();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadWriteSetup : %d\n", fIsError);
                retval = fIsError;				
                ErrorTrap(fIsError);
	         return fIsError;				
            }

            fIsError = fReadByteLoop();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadByteLoop : %d\n", fIsError);
                retval = fIsError;				
                ErrorTrap(fIsError);
	         return fIsError;				
            }
        }
//    #endif // end verify
    } // nRcvError

    CYTTSP_DBG(">>>> Program security data <<<<\n");

    #if 1

        //=======================================================//
        // Program security data into target PSoC. In the final application this
        // data should come from the HEX output of PSoC Designer.

        abSecurityDataPtr = 0;
        for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)
        {
            //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1
            fIsError = fReadWriteSetup();
            if (fIsError)
            {
                CYTTSP_DBG("error - fReadWriteSetup : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;				
            }
            // Load one bank of security data from hex file into buffer
            fIsError = fLoadSecurityData(bBankCounter);
            if (fIsError)
            {
                CYTTSP_DBG("error - fLoadSecurityData : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;				
            }
            // Secure one bank of the target flash
            fIsError = fSecureTargetFlash();
            if (fIsError)
            {
                CYTTSP_DBG("error - fSecureTargetFlash : %d\n", fIsError);
                retval = fIsError;
                ErrorTrap(fIsError);
	         return fIsError;				
            }
        }
    #endif
	
    CYTTSP_DBG(">>>> checksum <<<<\n");

    #if 1   // checksum
        //Doing Checksum
        iChecksumTarget = 0;
        fIsError = fAccTargetBankChecksum(&iChecksumTarget);
   
    if(nRcvError == 0)
    {
        if (fIsError)
        {
            CYTTSP_DBG("error - fAccTargetBankChecksum : %d\n", fIsError);
            retval = fIsError;
            ErrorTrap(fIsError);
	     return fIsError;
        }

        iChecksumData = iChecksumData & 0xFFFF;  //jpmoks : to be deleted.
        if (iChecksumTarget != iChecksumData)
        {
            CYTTSP_DBG("error - checksum is different\n");
            retval = fIsError;
            ErrorTrap(CHECKSUM_ERROR);
	     return fIsError;
        }
    }
    #endif

    // *** SUCCESS ***
    // At this point, the Target has been successfully Initialize, ID-Checked,
    // Bulk-Erased, Block-Loaded, Block-Programmed, Block-Verified, and Device-
    // Checksum Verified.

    // You may want to restart Your Target PSoC Here.
    ReStartTarget();
    CYTTSP_DBG(">>>> firmware update success <<<<\n");
    return retval;
}
// end of main()

#endif  //(PROJECT_REV_)
// end of file main.c
