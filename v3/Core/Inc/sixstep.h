/*
 * sixstep.h
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#ifndef INC_SIXSTEP_H_
#define INC_SIXSTEP_H_

#include <stdint.h>
#include "stm32f031x6.h"
#include <stdio.h>
#include <string.h>


void sixstep_startup();
void sixstep_loop();
void sixstep_slowloop();



#endif /* INC_SIXSTEP_H_ */
