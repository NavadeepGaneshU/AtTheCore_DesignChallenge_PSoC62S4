/*
 * ledbar.h
 *
 *  Created on: 16-Apr-2023
 *      Author: Navadeep
 */

#ifndef LEDBAR_H_
#define LEDBAR_H_

#define ALS_THRESH_LEVEL1   10
#define ALS_THRESH_LEVEL2   20
#define ALS_THRESH_LEVEL3   30
#define ALS_THRESH_LEVEL4   40
#define ALS_THRESH_LEVEL5   50
#define ALS_THRESH_LEVEL6   60
#define ALS_THRESH_LEVEL7   70
#define ALS_THRESH_LEVEL8   80
#define ALS_THRESH_LEVEL9   90
#define ALS_THRESH_LEVEL10  100

typedef enum
{
    eALSLevel1 = 0,
    eALSLevel2,
    eALSLevel3,
    eALSLevel4,
    eALSLevel5,
    eALSLevel6,
    eALSLevel7,
    eALSLevel8,
    eALSLevel9,
	eALSLevel10,
    eALSLevelMax
}TeALSIndicateLevels;

typedef enum
{
    eLed1 = 0,
    eLed2,
    eLed3,
    eLed4,
	eLed5,
	eLed6,
	eLed7,
	eLed8,
	eLed9,
	eLed10,
    eLedCountMax
}TeALSLed_t;

typedef enum
{
    eLedStateOff = 0,
    eLedStateStatic,
    eLedMaxStates
}TeLedState_t;

#endif /* LEDBAR_H_ */
