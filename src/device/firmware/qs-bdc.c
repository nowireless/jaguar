//*****************************************************************************
//
// qs-bdc.c - Brushed DC motor controller application.
//
// Copyright (c) 2008-2011 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 8264 of the RDK-BDC Firmware Package.
//
//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "shared/can_proto.h"
#include "adc_ctrl.h"
#include "button.h"
#include "can_if.h"
#include "controller.h"
#include "encoder.h"
#include "fan.h"
#include "hbridge.h"
#include "led.h"
#include "limit.h"
#include "param.h"
#include "pins.h"
#include "servo_if.h"
#include "wdog.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Brushed DC Motor Controller (qs-bdc)</h1>
//!
//! This application controls a brushed DC motor.  Two communication methods
//! are supported; a hobby servo-style input for basic voltage control or a CAN
//! input for more advanced control (the two inputs are mutually exclusive).
//!
//! When using either communication methods, a basic voltage control mode is
//! available.  In this mode, the external controller directly specifies the
//! desired output voltage.  When using CAN, an output voltage slew rate can be
//! specified, which results in the output voltage adjusting in a linear
//! fashion from the current voltage to the new voltage (as opposed to directly
//! jumping to the new voltage if the slew rate is disabled).
//!
//! Additional advanced control methods are also available when using the CAN
//! communication interface.  There are voltage compensation control mode,
//! current control mode, speed control mode, and position control mode.  Each
//! of these modes is mutually exclusive and operate using a PID controller
//! whose gains are fully programmable via the CAN interface.  Each PID
//! controller starts with all of its gains set to zero, so no output voltage
//! will be generated by any of these modes until the PID controller is at
//! least partially configured.
//!
//! In voltage compensation control mode, the output duty cycle is adjusted to
//! compensate for changes in the input voltage, resulting in a constant
//! voltage output.
//!
//! In speed control mode, the speed of the motor is measured using the
//! quadrature encoder input.  Only PHA is used for measuring the speed, so it
//! can be used with gear tooth sensors as well (which only provide a single
//! pulse stream, not a quadrature pair as provided by a shaft encoder).
//!
//! In position control mode, the position of the motor can be measured using
//! the quadrature encoder input or the analog input.  When using the analog
//! input, a 10K potentiometer must be coupled to the output shaft of the motor
//! (either before or after gearing) in some manner so that the motor position
//! can be tracked.
//!
//! The status of the motor controller can also be monitored over the CAN
//! interface.  The bus voltage, output voltage, motor current, ambient
//! temperature, speed, position, limit switch values, fault status, power
//! status, and firmware version can all be queried.
//!
//! This version of the firmware contains enhancements not present in the
//! pre-programmed firmware on the MDL-BDC.  These enhancements are:
//!
//! - Addition of the System Halt, System Reset, and System Resume commands.
//!
//! - Addition of the Maximum Output Voltage command to allow motors with lower
//!   voltage ratings (such as 7.2V motors) to be used.
//!
//! - Addition of the ability to read the value of the motor controller's
//!   parameters.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// This function is used to safely call the boot loader and will never return.
//
//*****************************************************************************
void
CallBootloader(void)
{
    //
    // Disable all processor interrupts.  Instead of disabling them one
    // at a time (and possibly missing an interrupt if new sources are
    // added), a direct write to NVIC is done to disable all peripheral
    // interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;

    //
    // Clear any active interrupts.  If the boot loader uses any
    // interrupts, they will therefore respond as expected.
    //
    HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_VECT_CLR_ACT;

    //
    // Set the HBridge to a safe state before the firmware update
    // starts.
    //
    HBridgeFirmwareUpdate();

    //
    // Set the LED to indicate that a firmware update is occurring.
    //
    LEDFirmwareUpdate();

    //
    // Call the boot loader.
    //
    (*((void (*)(void))(*(unsigned long *)0x2c)))();

    //
    // Loop forever.  This should never be reached.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the detection delay for determining what hardware this firmware is
// running on.  Since running the wrong firmware on the wrong hardware can be
// harmful this detection is important.
// Since each increment of SysCtlDelay() is 3 clocks the math to calculate
// SysCtlDelay units to microseconds is as follows:
//
// SysCtlDelay units/us = (16,000,000/3)/1,000,000
// SysCtlDelay units/us = 16/3 = 5.3
//
// Since the measured rise time on this unloaded pin was around 5us the delay
// needed is 5 times this value or 26.5. Now to be conservative this value is
// multiplied by 2 which result is an overall SysCtlDelay units of 53.
//
//*****************************************************************************
#define DETECTION_DELAY     (53)

//*****************************************************************************
//
// This function will determine if the firmware is running on the "old" Jaguar
// or the new Jaguar model.  This is is done by enabling an internal weak
// pull-up on PORTD pin 3 and reading back the value after a short delay.  On
// the "old" Jaguar there is a voltage divider network that will prevent the
// pin from being pulled up and the new Jaguar has this pin as a no connect and
// will pull up.
//
// The function will return 1 if this firmware is compatible with the hardware
// and 0 if the firmware is not compatible with the hardware.
//
//*****************************************************************************
static int
CheckFirmwareVersion(void)
{
    //
    // Get Hardware version.
    //
    ROM_GPIODirModeSet(GPIO_PORTD_BASE , GPIO_PIN_3, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPU);

    //
    // Delay to allow the GPIO pull-up a chance to have an effect.
    //
    SysCtlDelay(DETECTION_DELAY);

    //
    // Disable the pull-up.
    //
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA,
            GPIO_PIN_TYPE_STD);

    //
    // Read the state of the GPIO port D pin 3.
    //
    if(ROM_GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3) == GPIO_PIN_3)
    {
        //
        // If the value is 1 then the hardware was the newer jaguar module
        // that has nothing connected to this pin.
        //
        g_ucHardwareVersion = LM_HWVER_JAG_2_0;

        return(0);
    }

    //
    // If the value is 0 then the hardware was the older jaguar module
    // that has a voltage divider on this pin.
    //
    g_ucHardwareVersion = LM_HWVER_JAG_1_0;

    return(1);
}

//*****************************************************************************
//
// Initializes the various modules that make up the BDC motor controller and
// then spin in a sleep loop since all processing is handled in interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run from the main oscillator.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the peripherals used by this application.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    //
    // Make sure that this firmware matches the hardware.
    //
    if(CheckFirmwareVersion() == 0)
    {
        //
        // Initial configuration of the CAN network interface.
        //
        CANIFInit();

        //
        // Call the boot loader because this the wrong firmware type for this
        // hardware.
        //
        CallBootloader();
    }

    //
    // Set the priorities of the interrupts.  The following interrupts are used
    // (listed in priority from highest to lowest):
    //
    //     Watchdog    - Detects fault conditions due to a lack of input
    //                   control signal (servo or CAN).  This is the highest
    //                   priority since its occurrence should be immediately
    //                   handled, regardless of whatever else may be happening.
    //
    //     Servo input - The PWM input control signal.  This is next highest
    //                   priority since delays in handling this interrupt will
    //                   result in "jitter" in the detected input pulse width.
    //
    //     QEI         - The input from the quadrature encoder.  This is the
    //                   same priority as the servo input since CAN must be
    //                   used in order to use the encoder (in other words, QEI
    //                   and servo are mutually exclusive), and delays in
    //                   handling the QEI input interrupt will result in
    //                   "jitter" in the measured speed.
    //
    //     ADC0        - The completion of an ADC sampling.
    //
    //     PWM2        - The periodic control loop for the application.
    //
    //     CAN0        - The CAN control traffic.
    //
    ROM_IntPrioritySet(INT_WATCHDOG, 0x00);
    ROM_IntPrioritySet(SERVO_INT, 0x20);
    ROM_IntPrioritySet(QEI_PHA_INT, 0x20);
    ROM_IntPrioritySet(INT_ADC0SS0, 0x40);
    ROM_IntPrioritySet(INT_PWM0_2, 0x60);
    ROM_IntPrioritySet(INT_CAN0, 0x80);

    //
    // Initialize the controller.
    //
    ControllerInit();

    //
    // Enable the LED interface.
    //
    LEDInit();

    //
    // Get the initial configuration from flash if present.
    //
    ParamInit();

    //
    // Enable the button.
    //
    ButtonInit();

    //
    // Enable the limit switch inputs.
    //
    LimitInit();

    //
    // Initialize the fan.
    //
    FanInit();

    //
    // Configure the ADC.
    //
    ADCInit();

    //
    // Enable the H-Bridge interface.
    //
    HBridgeInit();

    //
    // Enable the Servo (PWM) interface.
    //
    ServoIFInit();

    //
    // Configure the encoder interface.
    //
    EncoderInit();

    //
    // Configure and enable the watchdog interrupt.
    //
    WatchdogInit();

    //
    // Initial configuration of the CAN network interface.
    //
    CANIFInit();

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Loop forever.  All the work is done in interrupt handlers, so there is
    // nothing to be done here.  If there is background work to be done with
    // any processor cycles left over after controlling the motor, then it can
    // be placed here.
    //
    while(1)
    {
        //
        // Put the processor to sleep until the next interrupt.
        //
#ifndef DEBUG
        SysCtlSleep();
#endif
    }
}
