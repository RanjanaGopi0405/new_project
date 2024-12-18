//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
//! \addtogroup driver_example_list
//! <h1>Empty Project Example</h1> 
//!
//! This example is an empty project setup for Driverlib development.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
#include "RfRemote.h"
//
// Main
//
 int rfSignal;
volatile RFSignalContext RfAsGpio;
volatile RemoteControlContext RfRemote;
Rf_RxHandle_t Rf_RxHandle={0};
__interrupt void epwm0ISR(void);

void main(void)
{
    initializeRfContextANDRemoteContext(&RfAsGpio,&RfRemote,&Rf_RxHandle);
    Device_init();
        Device_initGPIO();
        Interrupt_initModule();
        Interrupt_initVectorTable();


        Interrupt_register(INT_EPWM1, &epwm0ISR);

        SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
        Board_init();

     //   GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);

        EPWM_setActionQualifierAction(myEPWM0_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(myEPWM0_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
        Interrupt_enable(INT_EPWM1);

        EINT;
        ERTM;


    while(1)
    {

    }
}

__interrupt void epwm0ISR(void)
{

     rfSignal= GpioDataRegs.GPADAT.bit.GPIO0;
    RFSignalReadingAsGpio(&RfAsGpio,rfSignal);
    RfDataValidataion(&RfAsGpio,&RfRemote);
    rf_rx_routine(&RfAsGpio,&Rf_RxHandle,&RfRemote);
}
//
// End of File
//
