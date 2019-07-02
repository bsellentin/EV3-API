/** \file ev3_output.h
 * \brief Functions and constants for controlling EV3 outputs
 *
 * ev3_output.h contains declarations for the EV3 C API output functions.
 *
 * License:
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See the
 * License for the specific language governing rights and limitations
 * under the License.
 *
 * The Initial Developer of this code is John Hansen.
 * Portions created by John Hansen are Copyright (C) 2009-2013 John Hansen.
 * All Rights Reserved.
 *
 * ----------------------------------------------------------------------------
 *
 * \author John Hansen (bricxcc_at_comcast.net)
 * \date 2013-06-20
 * \version 1
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ev3_output_h
#define ev3_output_h

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <limits.h>

#include "ev3_command.h"
#include "ev3_constants.h"

/** @addtogroup OutputModuleFunctions
 * @{
 */

bool OutputInit(void);

bool OutputOpen(void);

bool OutputClose(void);

bool OutputExit(void);

bool OutputProgramStop(void);

bool OutputInitialized(void);

/** Stop outputs (brake or coast)
 *
 */
bool OutputStop(uint8_t Outputs, bool useBrake);

// Set output device type
bool OutputSetType(uint8_t Output, int8_t DeviceType);

bool OutputSetTypesArray(int8_t *pTypes);

bool OutputSetTypes(int8_t OutputA, int8_t OutputB, int8_t OutputC, int8_t OutputD);

/** Reset position  (POS=0)
 *
 */
bool OutputReset(uint8_t Outputs);

/** Set speed (relative to polarity - 
 * enables regulation if the output has a tachometer)
 */
bool OutputSpeed(uint8_t Outputs, int8_t Speed);

/** Set power (suspend regulation and positioning)
 *
 */
bool OutputPower(uint8_t Outputs, int8_t Power);

/** Starts outputs from present values
 *
 */
bool OutputStartEx(uint8_t Outputs, uint8_t Owner);

/** Starts outputs from present values
 *
 */
#define OutputStart(outputs) OutputStartEx((outputs), OWNER_NONE)

/** Set polarity (0=toggle)
 *
 */
bool OutputPolarity(uint8_t Outputs, int8_t Polarity);

/** Read actual speed and steps from last reset
 */
bool OutputRead(uint8_t Output, int8_t * Speed, int * TachoCount, int * TachoSensor);

//bool OutputReady(uint8_t Outputs, uint8_t * Busy, uint8_t Owner);

/** check whether any of the outputs are busy
 */
bool OutputTest(uint8_t Outputs, bool * isBusy);

/** set bitmask of all the busy outputs
 */
bool OutputState(uint8_t Outputs, uint8_t * State);

/** Clears the tacho count used when in sensor mode
 */
bool OutputClearCount(uint8_t Outputs);

/** Gets the tacho count used in sensor mode
 */
bool OutputGetCount(uint8_t Output, int * Tacho);

/** Read steps from last reset
 */
bool OutputGetTachoCount(uint8_t Output, int * Tacho);

/** Read actual speed
 */
bool OutputGetActualSpeed(uint8_t Output, int8_t * Speed);

bool OutputStepPowerEx(uint8_t Outputs, int8_t Power, int Step1, int Step2, int Step3, bool useBrake, uint8_t Owner);

/** This function enables specifying a full motor power cycle in tacho counts.
 */
#define OutputStepPower(outputs, power, step1, step2, step3) OutputStepPowerEx((outputs), (power), (step1), (step2), (step3), TRUE, OWNER_NONE)

/** This function enables specifying a full motor power cycle in time.
 */
bool OutputTimePowerEx(uint8_t Outputs, int8_t Power, int Time1, int Time2, int Time3, bool useBrake, uint8_t Owner);

/** This function enables specifying a full motor power cycle in time.
 */
#define OutputTimePower(outputs, power, time1, time2, time3) OutputTimePowerEx((outputs), (power), (time1), (time2), (time3), TRUE, OWNER_NONE)

bool OutputStepSpeedEx(uint8_t Outputs, int8_t Speed, int Step1, int Step2, int Step3, bool useBrake, uint8_t Owner);

/** This function enables specifying a full motor power cycle in tacho counts.
 * The system will automatically adjust the power level to the motor to keep
 * the specified output speed.
 * @param outputs Outputs. See \ref OutputPortConstants.
 * @param speed
 * @param step1 Specifyes the power ramp up periode in tacho counts.
 * @param step2 Specifyes the constant power period in tacho counts.
 * @param step3 Specifyes the power down period in tacho counts.
 */
#define OutputStepSpeed(outputs, speed, step1, step2, step3) OutputStepSpeedEx((outputs), (speed), (step1), (step2), (step3), TRUE, OWNER_NONE)

bool OutputTimeSpeedEx(uint8_t Outputs, int8_t Speed, int Time1, int Time2, int Time3, bool useBrake, uint8_t Owner);

/** This function enables specifying a full motor power cycle in time.
 */
#define OutputTimeSpeed(outputs, speed, time1, time2, time3) OutputTimeSpeedEx((outputs), (speed), (time1), (time2), (time3), TRUE, OWNER_NONE)

bool OutputStepSyncEx(uint8_t Outputs, int8_t Speed, short Turn, int Step, bool useBrake, uint8_t Owner);

/** This function enables synchronizing two motors. Synchronization should be
 * used when motors should run as synchrone as possible, for example to
 * archieve a model driving straight. Duration is specified in tacho counts.
 */
#define OutputStepSync(outputs, speed, turn, step) OutputStepSyncEx((outputs), (speed), (turn), (step), TRUE, OWNER_NONE)

bool OutputTimeSyncEx(uint8_t Outputs, int8_t Speed, short Turn, int Time, bool useBrake, uint8_t Owner);

/** This function enables synchronizing two motors. Synchronization should be
 * used when motors should run as synchrone as possible, for example to
 * archieve a model driving straight. Duration is specified in time.
 */
#define OutputTimeSync(outputs, speed, turn, time) OutputTimeSyncEx((outputs), (speed), (turn), (time), TRUE, OWNER_NONE)

void SetOutputEx(uint8_t Outputs, uint8_t Mode, uint8_t reset);

#define SetOutput(outputs, mode) SetOutputEx((outputs), (mode), RESET_NONE)

/** Set direction
 * @param Outputs
 * @param Dir Direction. See \ref OutDirConstants.
 */
void SetDirection(uint8_t Outputs, uint8_t Dir);

/** Negative values forward, positive values backwards
 */
void SetPower(uint8_t Outputs, int8_t Power);

void SetSpeed(uint8_t Outputs, int8_t Speed);

void OnEx(uint8_t Outputs, uint8_t reset);

void OffEx(uint8_t Outputs, uint8_t reset);

void FloatEx(uint8_t Outputs, uint8_t reset);

#define On(outputs) OnEx((outputs), RESET_NONE)

#define Off(outputs) OffEx((outputs), RESET_NONE)

#define Float(outputs) FloatEx((outputs), RESET_NONE)

#define Coast(outputs) FloatEx((outputs), RESET_NONE)

void Toggle(uint8_t Outputs);

void Fwd(uint8_t Outputs);

void Rev(uint8_t Outputs);

void OnFwdEx(uint8_t Outputs, int8_t Power, uint8_t reset);

void OnRevEx(uint8_t Outputs, int8_t Power, uint8_t reset);

/** Run motors forward with given power.
 */
#define OnFwd(outputs, power) OnFwdEx((outputs), (power), RESET_NONE)

/**Run motors backwards with given power.
 */
#define OnRev(outputs, power) OnRevEx((outputs), (power), RESET_NONE)

void OnFwdRegEx(uint8_t Outputs, int8_t Speed, uint8_t RegMode, uint8_t reset);

void OnRevRegEx(uint8_t Outputs, int8_t Speed, uint8_t RegMode, uint8_t reset);

#define OnFwdReg(outputs, speed) OnFwdRegEx((outputs), (speed), OUT_REGMODE_SPEED, RESET_NONE)

#define OnRevReg(outputs, speed) OnRevRegEx((outputs), (speed), OUT_REGMODE_SPEED, RESET_NONE)

void OnFwdSyncEx(uint8_t Outputs, int8_t Speed, short Turn, uint8_t reset);

void OnRevSyncEx(uint8_t Outputs, int8_t Speed, short Turn, uint8_t reset);

#define OnFwdSync(outputs, speed) OnFwdSyncEx((outputs), (speed), 0, RESET_NONE)

#define OnRevSync(outputs, speed) OnRevSyncEx((outputs), (speed), 0, RESET_NONE)

void RotateMotorNoWaitEx(uint8_t Outputs, int8_t Speed, int Angle, short Turn, bool Sync, bool Stop);

#define RotateMotorNoWait(outputs, speed, angle) RotateMotorNoWaitEx((outputs), (speed), (angle), 0, TRUE, TRUE)

void RotateMotorEx(uint8_t Outputs, int8_t Speed, int Angle, short Turn, bool Sync, bool Stop);

#define RotateMotor(outputs, speed, angle) RotateMotorEx((outputs), (speed), (angle), 0, TRUE, TRUE)

void OnForSyncEx(uint8_t Outputs, int Time, int8_t Speed, short Turn, bool Stop);

#define OnForSync(outputs, time, speed) OnForSyncEx((outputs), (time), (speed), 0, TRUE)

void OnForEx(uint8_t Outputs, int Time, int8_t Power, uint8_t reset);

#define OnFor(outputs, time, power) OnForEx((outputs), (time), (power), RESET_NONE)

/** This function enables resetting the tacho count for the individual output ports.
 * The tacho count is also resetted at program start.
 */
void ResetTachoCount(uint8_t Outputs);

void ResetBlockTachoCount(uint8_t Outputs);

/** This function enables the program to clear the tacho count used as sensor input.
 * This rotation count is resetted at boot time, not at program start.
 */
void ResetRotationCount(uint8_t Outputs);

/** Resets tacho and rotation count.
 */
void ResetAllTachoCounts(uint8_t Outputs);

void ResetCount(uint8_t Outputs, uint8_t reset);

/** This function enables reading current output tacho count in degrees.
 * This count is set to 0 at program start.
 * See also: ResetTachoCount()
 */
int MotorTachoCount(uint8_t Output);

int MotorBlockTachoCount(uint8_t Output);

int8_t MotorPower(uint8_t Output);

int8_t MotorActualSpeed(uint8_t Output);

/** This function enables the program to read the tacho count in degrees as
 * sensor input. This count is set to 0 at boot time, not at program start.
 */
int MotorRotationCount(uint8_t Output);

/** This function enables the program to test if a output port is busy.
 * @returns 1 if output is busy, 0 if not.
 */
bool MotorBusy(uint8_t Output);


//uint8_t MotorGetBusyFlags(void);
//void MotorSetBusyFlags(uint8_t Flags);
//void ResetDelayCounter(uint8_t Pattern);

/** @} */  // end of OutputModuleFunctions

#endif // ev3_output_h

#ifdef __cplusplus
}
#endif
