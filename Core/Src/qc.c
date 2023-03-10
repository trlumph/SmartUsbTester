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
	HAL_GPIO_WritePin(DP_H_GPIO_Port, DP_H_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DP_L_GPIO_Port, DP_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DP_06V(){
	HAL_GPIO_WritePin(DP_H_GPIO_Port, DP_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DP_L_GPIO_Port, DP_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DP_33V(){
	HAL_GPIO_WritePin(DP_H_GPIO_Port, DP_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DP_L_GPIO_Port, DP_L_Pin, GPIO_PIN_SET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DM_0V(){
	HAL_GPIO_WritePin(DM_H_GPIO_Port, DM_H_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DM_L_GPIO_Port, DM_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DM_06V(){
	HAL_GPIO_WritePin(DM_H_GPIO_Port, DM_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DM_L_GPIO_Port, DM_L_Pin, GPIO_PIN_RESET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void DM_33V(){
	HAL_GPIO_WritePin(DM_H_GPIO_Port, DM_H_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DM_L_GPIO_Port, DM_L_Pin, GPIO_PIN_SET);
	__qc_state = QC_MANUAL_UNDEFINED;
}

void Init_5V(){
	DP_06V();
	HAL_Delay(1500);
	DM_0V();
	HAL_Delay(3);
	__qc_state = QC_5V;
}

void Init_9V(){
	DP_33V();
	HAL_Delay(1500);
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
	HAL_Delay(3);
	DP_06V();
	HAL_Delay(60);
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

void _setPinToRead(uint16_t pin){
	/*Configure GPIO pins to READ*/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void _setPinToWrite(uint16_t pin){
	/*Configure GPIO pins to WRITE : DM_H_Pin DP_H_Pin*/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int ReadPin(uint16_t pin){
	_setPinToRead(pin);
	int value = HAL_GPIO_ReadPin(GPIOB, pin);
	_setPinToWrite(pin);
	return value;
}

qc_support_t HasQCSupport(){
	_setPinToRead(DM_H_Pin|DM_L_Pin|DP_L_Pin|DP_H_Pin);
	_setPinToWrite(DP_L_Pin|DP_H_Pin);

	HAL_Delay(50);

	DP_33V();
	HAL_Delay(1);
	int firstShorted = ReadPin(DM_H_Pin);
	
	DP_06V();

	HAL_Delay(1500);
	DP_33V();
	HAL_Delay(1);
	int thanReleased = !ReadPin(DM_H_Pin);

	_setPinToWrite(DM_H_Pin|DM_L_Pin|DP_L_Pin|DP_H_Pin);
	HAL_Delay(50);
	DP_06V();
	DM_0V();
	HAL_Delay(1);
	if(firstShorted && thanReleased){
		__qc_state = QC_5V;
		return QC2_PLUS;
	}
	__qc_state = QC_MANUAL_UNDEFINED;
	return QC_NOT_SUPPORTED;
}