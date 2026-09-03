#ifndef __BSP_DSHOT_H
#define __BSP_DSHOT_H

#include <stdint.h>

//dshot命令,设置命令时需要将遥测标志位置1
enum{
	DSHOT_CMD_MOTOR_STOP = 0,		// Currently not implemented
	DSHOT_CMD_BEEP1,			// Wait at least length of beep (380ms) before next command
	DSHOT_CMD_BEEP2,			// Wait at least length of beep (380ms) before next command
	DSHOT_CMD_BEEP3,			// Wait at least length of beep (400ms) before next command
	DSHOT_CMD_BEEP4,			// Wait at least length of beep (400ms) before next command
	DSHOT_CMD_BEEP5,			// Wait at least length of beep (400ms) before next command
	DSHOT_CMD_ESC_INFO, 			// Currently not implemented
	DSHOT_CMD_SPIN_DIRECTION_1,		// Need 6x, no wait required
	DSHOT_CMD_SPIN_DIRECTION_2,		// Need 6x, no wait required
	DSHOT_CMD_3D_MODE_OFF,		// Need 6x, no wait required
	DSHOT_CMD_3D_MODE_ON, 		// Need 6x, no wait required
	DSHOT_CMD_SETTINGS_REQUEST, 		// Currently not implemented
	DSHOT_CMD_SAVE_SETTINGS, 		// Need 6x, wait at least 12ms before next command
	DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,	// Need 6x, no wait required
	DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,	// Need 6x, no wait required
	DSHOT_CMD_LED0_ON,			// Currently not implemented
	DSHOT_CMD_LED1_ON,			// Currently not implemented
	DSHOT_CMD_LED2_ON,			// Currently not implemented
	DSHOT_CMD_LED3_ON,			// Currently not implemented
	DSHOT_CMD_LED0_OFF,			// Currently not implemented
	DSHOT_CMD_LED1_OFF,			// Currently not implemented
	DSHOT_CMD_LED2_OFF,			// Currently not implemented
	DSHOT_CMD_LED3_OFF,			// Currently not implemented
	DSHOT_CMD_MAX = 47
};


typedef struct{
	uint8_t Telemetry; //遥测标志位
	short throttle; //油门值
}dshotMotorVal_t;

typedef struct {
    void (*init)(void);
    void (*set_target)(dshotMotorVal_t m1,dshotMotorVal_t m2,dshotMotorVal_t m3,dshotMotorVal_t m4);
}MotorInterface_t,*pMotorInterface_t;

extern MotorInterface_t UserDshotMotor;


#endif /* __BSP_DSHOT_H */
