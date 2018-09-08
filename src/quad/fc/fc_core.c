
#include <stdio.h>		// testing purposes
#include "rx.h"			// including time.h
#include "debug.h"
#include "system.h"
#include "pwm_output.h"
#include "mixer.h"

uint8_t motorControlEnable = false;

bool isRXDataNew;

void processRx(timeUs_t currentTimeUs)
{
	calculateRxChannelsAndUpdateFailsafe(currentTimeUs);
}

static void subTaskMotorUpdate(void)
{
	uint32_t startTime;
	if (debugMode == DEBUG_CYCLETIME) {
		startTime = micros();
		static uint32_t previousMotorUpdateTime;		// static keyword to keep the previous motor update time
		const uint32_t currentDeltaTime = startTime - previousMotorUpdateTime;
		debug[2] = currentDeltaTime;
//		debug[3] = currentDeltaTime - targetPidLooptime;		// TODO: targetPidLooptime is defined in pid.c
		previousMotorUpdateTime = startTime;
	}else if (debugMode == DEBUG_PIDLOOP) {
		startTime = micros();
	}
	
	mixTable();			// TODO: add &currentProfile->pidProfile later
//	mixTable(&currentProfile->pidProfile);
	
	if (motorControlEnable) {
//		printf("motorControlEnable: %s, %d\r\n", __FUNCTION__, __LINE__);
		writeMotors();				// uncomment pwmCompleteMotorUpdate() function later in the writeMotors() function
	}
}

void taskMainPidLoop(timeUs_t currentTimeUs)
{
	/* gyroUpdate */
	
	
	/* subTaskPidController */


	/* subTaskMotorUpdate */
	subTaskMotorUpdate();
}
