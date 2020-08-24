/*
 * STM32F103_VL53L0X.h
 *
 *  Created on: 15 may. 2020
 *      Author: Pedro Fontana
 */

#ifndef STM32F103_VL53L0X_H_
#define STM32F103_VL53L0X_H_

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <stdbool.h>

#include "STM32F103_UART.h"
#include "STM32F103_delay.h"

/*
 *	VL53L0X_Ranging_Mode_t
 * 		VL53L0X has 3 possible ranging modes (p.13 02904 Rev 2):
 * 			Single Ranging:  Ranging is perform only once after the API call is made
 * 			Continuous Ranging : As soon as the measurement is finished, another is started without delay
 * 			Timed Ranging: After the measurement is finished, another one is started after a user defined delay
 * 		For my project, I only use continuous ranging and simple ranging
 */
#define IS_VL53L0X_DEVICE_MODE(MODE)	((MODE == VL53L0X_DEVICEMODE_SINGLE_RANGING ) || (MODE == VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) \
										(MODE == VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM ) || (MODE == VL53L0X_DEVICEMODE_GPIO_OSC )  		 \
										(MODE == VL53L0X_DEVICEMODE_SINGLE_ALS ) || (MODE == VL53L0X_DEVICEMODE_GPIO_DRIVE)  			 \
										(MODE == VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING))

/* No range measured (distance further than max tolerable)*/
#define VL53L0X_ERROR_INVALID_RANGE		((VL53L0X_Error)	-13)

/* Private functions */
void print_pal_error(VL53L0X_Error Status);
void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev);
VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev);

/* Public functions */
VL53L0X_Error VL53L0XCalibrate(VL53L0X_Dev_t *pMyDevice);
VL53L0X_Error VL53L0XInit(VL53L0X_Dev_t *pMyDevice, VL53L0X_DeviceModes mode);
VL53L0X_Error VL53L0XGetMeasures(VL53L0X_Dev_t *pMyDevice, uint16_t *measurements,  uint32_t* io_no_meas);
VL53L0X_Error VL53L0XGetSingleMeasure(VL53L0X_Dev_t *pMyDevice, uint16_t *measure);

#endif /* STM32F103_VL53L0X_H_ */
