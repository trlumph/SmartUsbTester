/*
 * qc.h
 *
 *  Created on: Dec 21, 2022
 *      Author: tymur
 */

#ifndef INC_QC_H_
#define INC_QC_H_

typedef enum qc_t {
	QC_OFF,
	QC_5V,
	QC_9V,
	QC_12V,
	QC_20V,
	QC_CONTINUOUS,
	QC_MANUAL_UNDEFINED
} qc_t;

typedef enum qc_support_t {
	QC_NOT_SUPPORTED,
	QC2_PLUS,
	QC_UNKNOWN
} qc_support_t;

qc_t GetStateQC();

void DP_0V();

void DP_06V();

void DP_33V();

void DM_0V();

void DM_06V();

void DM_33V();

void Init_5V();

void Init_9V();

void Set_5V();

void Set_9V();

void Set_12V();

void Set_20V();

void ContinuousMode();

void IncVoltage();

void DecVoltage();

qc_support_t HasQCSupport();

#endif /* INC_QC_H_ */
