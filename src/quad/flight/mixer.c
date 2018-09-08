
#include <stdio.h>				// debugging purposes
#include "mixer.h"
#include "pwm_output.h"
#include "rx.h"
#include "maths.h"

static uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;

static motorMixer_t *customMixers;

static motorConfig_t *motorConfig;
static mixerConfig_t *mixerConfig;
rxConfig_t *rxConfig;

static uint16_t disarmMotorOutput;
static float rcCommandThrottleRange;
uint16_t motorOutputHigh, motorOutputLow;

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

/* throttle, roll, pitch, yaw */
static const motorMixer_t mixerQuadX[] = {
	{ 1.0f, -1.0f, 1.0f, -1.0f },				// REAR_R
	{ 1.0f, -1.0f, -1.0f, 1.0f },				// FRONT_R
	{ 1.0f, 1.0f, 1.0f, 1.0f },					// REAR_L
	{ 1.0f, 1.0f, -1.0f, -1.0f },				// FRONT_L
};

void mixerUseConfigs(motorConfig_t *motorConfigToUse, mixerConfig_t *mixerConfigToUse, rxConfig_t *rxConfigToUse)
{
	motorConfig = motorConfigToUse;
	mixerConfig = mixerConfigToUse;
	rxConfig = rxConfigToUse;
}

void mixerResetDisarmedMotors(void)
{
	/* set disarmed motor values */
	for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
		motor_disarmed[i] = disarmMotorOutput;			// disarmMotorOutput = motorConfig->mincommand = 1000
	}
}

void mixerConfigurationOutput(void)
{
	motorCount = QUAD_MOTOR_COUNT;
	
	for (int i = 0; i < motorCount; i++) {
		currentMixer[i] = mixerQuadX[i];
	}
	
	mixerResetDisarmedMotors();
}

uint8_t getMotorCount(void)
{
	return motorCount;
}

void writeMotors(void)
{
	if (pwmAreMotorsEnabled()) {
		for (int i = 0; i < motorCount; i++) {
#if 1
			if (i == 0)
				printf("motor 1 PWM control value: %u\r\n", motor[i]);
			else if (i == 1)
				printf("motor 2 PWM control value: %u\r\n", motor[i]);
			else if (i == 2)
				printf("motor 3 PWM control value: %u\r\n", motor[i]);
			else if (i == 3) {
				printf("motor 4 PWM control value: %u\r\n", motor[i]);
				printf("\r\n");
			}
#endif
			pwmWriteMotor(i, motor[i]);
		}
	}
	
	pwmCompleteMotorUpdate(motorCount);		// TODO: UNCOMMENT this line later
}

bool isMotorProtocolDshot(void)
{
#ifdef USE_DSHOT
	switch (motorConfig->motorPwmProtocol) {
		case PWM_TYPE_DSHOT1200:
		case PWM_TYPE_DSHOT600:
		case PWM_TYPE_DSHOT300:
		case PWM_TYPE_DSHOT150:
			return true;
		default:
			return false;
	}
#else
	return false;
#endif
}

/* Scaled ESC outputs */
void initEscEndpoints(void)
{
#ifdef USE_DSHOT
	/* TODO: DSHOT Esc initialisation implementation here */
	if (isMotorProtocolDshot()) {
		
	}else
#endif
	{
		disarmMotorOutput = motorConfig->mincommand;				// motorConfig->mincommand = 1000
		motorOutputLow = motorConfig->minthrottle;					// motorConfig->minthrottle = 1070
		motorOutputHigh = motorConfig->maxthrottle;					// motorConfig->maxthrottle = 2000
//		printf("disarmMotorOutput: %u\r\n", disarmMotorOutput);		// disarmMotorOutput = 1000
//		printf("motorOutputLow: %u\r\n", motorOutputLow);			// motorOutputLow = 1070
//		printf("motorOutputLow: %u\r\n", motorOutputHigh);			// motorOutputHigh = 2000
	}
	
	rcCommandThrottleRange = (PWM_RANGE_MAX - rxConfig->mincheck);	// PWM_RANGE_MAX = 2000, rxConfig->mincheck = 1100, rcCommandThrottleRange = 2000 - 1100 = 900
//	printf("rcCommandThrottleRange: %f\r\n", rcCommandThrottleRange);	// rcCommandThrottleRange = 900.000000
}

void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
	currentMixerMode = mixerMode;			// mixerMode = MIXER_QUADX = 3
	customMixers = initialCustomMixers;
	initEscEndpoints();
}

void mixTable(void)
//void mixTable(pidProfile_t *pidProfile)		// TODO: add pidProfile_t *pidProfile parameter later
{
	/* Scale roll/pitch/yaw uniformly to fit within throttle range */
	float throttle, currentThrottleInputRange = 0;
	uint16_t motorOutputMin, motorOutputMax;
	static uint16_t throttlePrevious = 0;			// store the last throttle direction for deadband transitions
	
//	printf("rcCommand[THROTTLE]: %d, %s, %d\r\n", rcCommand[THROTTLE], __FUNCTION__, __LINE__);
	throttle = rcCommand[THROTTLE] - rxConfig->mincheck;	// current rcCommand[THROTTLE] value - 1100 (mincheck)
	currentThrottleInputRange = rcCommandThrottleRange;		// rcCommandThrottleRange = 2000 - 1100 = 900
//	printf("throttle: %f, %s, %d\r\n", throttle, __FUNCTION__, __LINE__);		// throttle = [-100.000;892]
//	printf("currentThrottleInputRange: %f, %s, %d\r\n", currentThrottleInputRange, __FUNCTION__, __LINE__);
	
	/* Find min and max throttle based on condition */
	motorOutputMax = motorOutputHigh;		// motorOutputMax = motorOutputHigh = 2000 (max throttle)
	motorOutputMin = motorOutputMin;		// motorOutputMin = motorOutputMin = 1070 (min throttle)
	
	/* throttle / currentThrottleInputRange = [-0.1111;0.992222] */
//	printf("throttle / currentThrottleInputRange: %f, %s, %d\r\n", throttle / currentThrottleInputRange, __FUNCTION__, __LINE__);
	throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);		// throttle is between -0.111111 and 0.986667
//	printf("throttle after constrainf: %f, %s, %d\r\n", throttle, __FUNCTION__, __LINE__);
	
	const float motorOutputRange = motorOutputMax - motorOutputMin;		// motorOutputMax - motorOutputMin = 2000 - 1070 = 930
	
	/* Calculate and Limit the PIDsum */
	
	
	/* Calculate voltage compensation */
	
	
	/* TODO: modify this when finishing PID controllers */
	for (uint32_t i = 0; i < motorCount; i++) {
//		motor[i] = motorOutputMin;
//		printf("motor[%d]: %d\r\n", i, motor[i]);
		motor[i] = rcCommand[THROTTLE];
	}
}
