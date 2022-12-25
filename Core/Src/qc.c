/*
 * qc.c
 *
 *  Created on: Dec 21, 2022
 *      Author: tymur
 */
#include "qc.h"
#include "main.h"

qc_t __qc_state = QC_OFF;

qc_t GetStateQC(){
	return __qc_state;
}

void DisableQC(){
	Set_5V();
	__qc_state = QC_OFF;
}

void DP_0V(){
	HAL_GPIO_WritePin(GPIOE, DP_H_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, DP_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DP_06V(){
	HAL_GPIO_WritePin(GPIOE, DP_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, DP_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DP_33V(){
	HAL_GPIO_WritePin(GPIOE, DP_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, DP_L_Pin, GPIO_PIN_SET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DM_0V(){
	HAL_GPIO_WritePin(GPIOE, DM_H_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, DM_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DM_06V(){
	HAL_GPIO_WritePin(GPIOE, DM_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, DM_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DM_33V(){
	HAL_GPIO_WritePin(GPIOE, DM_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, DM_L_Pin, GPIO_PIN_SET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void Init_5V(){
	DP_06V();
	HAL_Delay(1250);
	DM_0V();
	HAL_Delay(3);
	__qc_state = QC_5V;
}

void Init_9V(){
	DP_33V();
	HAL_Delay(1250);
	DM_06V();
	HAL_Delay(3);
	__qc_state = QC_9V;
}

void Set_5V(){
	DP_06V();
	DM_0V();
	__qc_state = QC_5V;
}

void Set_9V(){
	DP_33V();
	DM_06V();
	__qc_state = QC_9V;
}

void Set_12V(){
	 DP_06V();
	 DM_06V();
	 __qc_state = QC_12V;
}

void Set_20V(){
	 DP_33V();
	 DM_33V();
	 __qc_state = QC_20V;
}

// This tells the device that it is entering continuous mode.
void ContinuousMode(){
	DM_33V();
	DP_06V();
	__qc_state = QC_CONTINUOUS;
}

void IncVoltage(){
	// Send 1 pulse

	//   -   1ms   1ms
	//      ******      //
	//      *    *      //
	//      *    *      //
	//*******    *******//

	DP_33V();
	HAL_Delay(1);
	DP_06V();
	HAL_Delay(1);
	__qc_state = QC_CONTINUOUS;
}

void DecVoltage(){
	// Send 1 pulse

	//   -   1ms   1ms
	//*******    *******//
	//      *    *      //
	//      *    *      //
	//      ******      //

	DM_06V();
	HAL_Delay(1);
	DM_33V();
	HAL_Delay(1);
	__qc_state = QC_CONTINUOUS;
}
