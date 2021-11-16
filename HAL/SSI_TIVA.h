/*
 * mySSI.h
 *
 *  Created on: Aug 27, 2018
 *      Author: Zhoubao Pang
 */

#ifndef SSI_TIVA_H_
#define SSI_TIVA_H_


#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

#include "../Encoder.h"


uint32_t abs_angle_0_prev;
uint32_t abs_angle_1_prev;

//Variables for velocity filtering
int32_t abs_angle_0_vel_f;
int32_t abs_angle_1_vel_f;

int32_t abs_angle_0_vel_prev;
int32_t abs_angle_1_vel_prev;

int32_t abs_angle_0_vel;
int32_t abs_angle_1_vel;

void SSI0_Gurley_Config(void);
void SSI0_Orbis_Config(void);
void SSI0_Disable(void);    // Not Tested!!

void SSI1_Gurley_Config(void);
void SSI1_Orbis_Config(void);
void SSI1_Disable(void);

void SSI2_Config(void);
void SSI2_Disable(void);    // Not Tested!!

void SSI3_Config_SPI(void);

/*
void readAbsEnc_0(Encoder* encoder);
void readAbsEnc_Vel_0(Encoder* encoder);
void readAbsEnc_1(Encoder* encoder);
void readAbsEnc_Vel_1(Encoder* encoder);
*/

#endif /* SSI_TIVA_H_ */
