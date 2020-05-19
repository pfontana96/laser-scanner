/*
 * STM32F103_VL53L0X.c
 *
 *  Created on: 15 may. 2020
 *      Author: Pedro Fontana
 */

#include "STM32F103_VL53L0X.h"

#define VL53L0X_LOG_INFO
//#define VL53L0X_LONG_RANGE
//#define VL53L0X_PERFORM_CALIBRATION

char UART_buffer[150];

#ifdef VL53L0X_PERFORM_CALIBRATION
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;
#else
// Stored values after calibration
uint32_t refSpadCount = 3;
uint8_t isApertureSpads = 1;
uint8_t VhvSettings = 31;
uint8_t PhaseCal = 1;
#endif

/* 		ST VL53L0X API example modified 		*/

VL53L0X_Error VL53L0XInit(VL53L0X_Dev_t *pMyDevice, VL53L0X_DeviceModes mode)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_DeviceInfo_t DeviceInfo;

	assert_param(IS_VL53L0X_DEVICE_MODE(mode));

	if(Status == VL53L0X_ERROR_NONE)
	{
	  UART2puts ("Call of VL53L0X_DataInit\r\n");
	  Status = VL53L0X_DataInit(pMyDevice); // Data initialization
	  print_pal_error(Status);
	}

	if(Status == VL53L0X_ERROR_NONE)
	{
	  Status = VL53L0X_GetDeviceInfo(pMyDevice, &DeviceInfo);
	}
	if(Status == VL53L0X_ERROR_NONE)
	{
	  UART2puts("VL53L0X_GetDeviceInfo:\r\n");
	  sprintf(UART_buffer, "VL53L0X_GetDeviceInfo:\r\n\tDevice Name : %s\r\n\tDevice Type : %s\r\n\t", DeviceInfo.Name, DeviceInfo.Type);
	  UART2puts(UART_buffer);
	  sprintf(UART_buffer, "Device ID : %s\r\n\tProductRevisionMajor : %d\r\n\tProductRevisionMinor : %d\r\n", DeviceInfo.ProductId, DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
	  UART2puts(UART_buffer);

	  if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
		sprintf(UART_buffer, "Error expected cut 1.1 but found cut %d.%d\r\n",
				DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
		UART2puts(UART_buffer);
		Status = VL53L0X_ERROR_NOT_SUPPORTED;
	  }
	}

	if(Status == VL53L0X_ERROR_NONE)
	{
		UART2puts("Call of VL53L0X_StaticInit\r\n\t");
		Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
		// StaticInit will set interrupt by default
		print_pal_error(Status);
	}
#ifdef VL53L0X_PERFORM_CALIBRATION
	if(Status == VL53L0X_ERROR_NONE)
	{
		UART2puts ("Call of VL53L0X_PerformRefCalibration\r\n\t");
		Status = VL53L0X_PerformRefCalibration(pMyDevice,
				&VhvSettings, &PhaseCal); // Device Initialization
		print_pal_error(Status);

		sprintf(UART_buffer, "\tVhvSettings: %i\r\n\tPhaseCal: %i\r\n", VhvSettings, PhaseCal);
		UART2puts(UART_buffer);
	}

	if(Status == VL53L0X_ERROR_NONE)
	{
		UART2puts ("Call of VL53L0X_PerformRefSpadManagement\r\n\t");
		Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
				&refSpadCount, &isApertureSpads); // Device Initialization
		print_pal_error(Status);

		sprintf(UART_buffer, "\trefSpadCount: %d\r\n\tisApertureSpads: %i\r\n", (int) refSpadCount, isApertureSpads);
		UART2puts(UART_buffer);
	}
	Status = VL53L0X_ERROR_NONE; // To fix error as stated in ST forum
#else
	if(Status == VL53L0X_ERROR_NONE)
	{
		UART2puts ("Call of VL53L0X_SetRefSpads\r\n\t");
		Status = VL53L0X_SetReferenceSpads(pMyDevice,
				refSpadCount, isApertureSpads); // Device Initialization
		print_pal_error(Status);
	}

	if(Status == VL53L0X_ERROR_NONE)
	{
		UART2puts ("Call of VL53L0X_SetRefCalibration\r\n\t");
		Status = VL53L0X_SetRefCalibration(pMyDevice,
				VhvSettings, PhaseCal); // Device Initialization
		print_pal_error(Status);
	}
#endif

	if(Status == VL53L0X_ERROR_NONE)
	{
		UART2puts ("Call of VL53L0X_SetDeviceMode\r\n\t");
		Status = VL53L0X_SetDeviceMode(pMyDevice, mode); // Setup in single ranging mode
		print_pal_error(Status);
	}

#ifdef VL53L0X_LONG_RANGE
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				(FixPoint1616_t)(0.1*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
				(FixPoint1616_t)(60*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
				33000);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
				VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
				VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	}
#endif


	return Status;
}

VL53L0X_Error VL53L0XGetMeasures(VL53L0X_Dev_t *pMyDevice, uint16_t *measurements, uint32_t* io_no_meas)
{
	uint32_t i;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	char rangeStatusStr[VL53L0X_MAX_STRING_LENGTH];
	uint32_t no_of_measurements = *io_no_meas;
	*io_no_meas = 0;

	UART2puts ("Call of VL53L0X_StartMeasurement\r\n\t");
	Status = VL53L0X_StartMeasurement(pMyDevice);
	print_pal_error(Status);

	if(Status == VL53L0X_ERROR_NONE)
	{
		for(i = 0; i < no_of_measurements; i++)
	   {
			Status = WaitMeasurementDataReady(pMyDevice);

			if(Status == VL53L0X_ERROR_NONE)
			{

			   Status = VL53L0X_GetRangingMeasurementData(pMyDevice, &RangingMeasurementData);
			   if((RangingMeasurementData.RangeStatus == 0 || RangingMeasurementData.RangeStatus == 1) &&
				   Status == VL53L0X_ERROR_NONE)
			   {
				   *(measurements + i) = RangingMeasurementData.RangeMilliMeter;
				   *io_no_meas = *io_no_meas + 1;
			   }

#ifdef VL53L0X_LOG_INFO
			   VL53L0X_GetRangeStatusString(RangingMeasurementData.RangeStatus, rangeStatusStr);
			   sprintf(UART_buffer, "Measure %d: %d mm (%s)\r\n", (int) i, (int) RangingMeasurementData.RangeMilliMeter,
					   rangeStatusStr);
			   UART2puts(UART_buffer);
#endif

			   // Clear the interrupt
			   VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			   VL53L0X_PollingDelay(pMyDevice);
			}
	   }
	}

	UART2puts ("Call of VL53L0X_StopMeasurement\r\n\t");
	Status = VL53L0X_StopMeasurement(pMyDevice);
	print_pal_error(Status);

	UART2puts ("Wait Stop to be competed\r\n\t");
	Status = WaitStopCompleted(pMyDevice);
	print_pal_error(Status);

	UART2puts("Clear Interrupt Mask\r\n\t");
	Status = VL53L0X_ClearInterruptMask(pMyDevice,
			VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	print_pal_error(Status);

	return Status;
}

VL53L0X_Error VL53L0XGetSingleMeasure(VL53L0X_Dev_t *pMyDevice, uint16_t *measure)
{
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	char rangeStatusStr[VL53L0X_MAX_STRING_LENGTH];

	Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
    		&RangingMeasurementData);

	*measure = RangingMeasurementData.RangeMilliMeter;

#ifdef VL53L0X_LOG_INFO
	VL53L0X_GetRangeStatusString(RangingMeasurementData.RangeStatus, rangeStatusStr);
	sprintf(UART_buffer, "Measure: %d mm (%s)\r\n", *measure, rangeStatusStr);
#endif

	return Status;
}


/*		ST VL53L0X API example 		*/

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    sprintf(UART_buffer, "Range Status: %i : %s\r\n", RangeStatus, buf);
    UART2puts(UART_buffer);

}

void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    sprintf(UART_buffer, "API Status: %i -> %s\r\n", Status, buf);
    UART2puts(UART_buffer);
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	DelayMs(3);
	return VL53L0X_ERROR_NONE;
}

