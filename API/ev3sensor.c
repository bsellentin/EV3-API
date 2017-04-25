 /*
 * EV3 Sensor API
 *
 * Copyright (C) 2014 Carsten Zeiffert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/
 *
 * ----------------------------------------------------------------------------
 *
 * \author Simón Rodriguez Perez(Hochschule Aschaffenburg)
 * \date 2015-02-28
 * \version 2
 * \note Correct readout for Touch-, Sonar- and Lightsensor
 *
 * ----------------------------------------------------------------------------
 *
 * \author Simón Rodriguez Perez
 * \date 2016-04-20
 * \version 3
 * \note Correct readout for Gyroscop and Infrared Sensor
 *
 * ----------------------------------------------------------------------------
 *
 * \author Bernd Sellentin
 * \date 2017
 * \version 3.1
 * \note Added NXT-sensors
 *
 */
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "ev3sensor.h"
#include "uart.h"
#include "iic.h"
#include "analog.h"
//#include "lms2012.h"

#include <stdint.h>

/***********************************/
// see "LEGO mindstorms ev3 firmware developer kit.pdf" chap.5 Device type list p.100ff
/**
 * \brief Identifier for sensor types and modes
 */
 
// EV3-TOUCH
#define TOUCH_TYPE 16
#define TOUCH_PRESS_MODE 0          // Press
#define TOUCH_BUMP_MODE 1           // count

// EV3-Light
#define COL_TYPE 29
#define COL_REFLECT_MODE 0          // Reflect
#define COL_AMBIENT_MODE 1          // Ambient
#define COL_COLOR_MODE 2            // Color

// EV3-Ultrasonic
#define US_TYPE 30
#define US_DIST_CM_MODE 0           // Dist in cm
#define US_DIST_MM_MODE 0           // Dist in mm
#define US_DIST_IN_MODE 1           // Dist in inch

// Gyroskop
#define GYRO_TYPE 32
#define GYRO_ANG_MODE 0             // angle
#define GYRO_RATE_MODE 1            // rate
#define GYRO_AR_MODE 3              // angle and rate

// Infrared
#define IR_TYPE 33
#define IR_PROX_MODE 0              // Proximity
#define IR_SEEK_MODE 1              // Seek
#define IR_REMOTE_MODE 2            // Remote Control

// IIC 
#define IIC_TYPE 100
#define IIC_BYTE_MODE 0

// NXT Temperature
#define NXT_TEMP_TYPE 6
#define NXT_TEMP_C_MODE 0           // Temperature in C
#define NXT_TEMP_F_MODE 1           // Temperature in F

// NXT Sensors
#define TYPE_NXT_TOUCH 1            //!< Device is NXT touch sensor lms2012.h Z566
#define NXT_TOUCH_MODE 0
#define TYPE_NXT_LIGHT 2            //!< Device is NXT light sensor
#define NXT_LIGHT_REFLECTED_MODE 0
#define NXT_LIGHT_AMBIENT_MODE 1
#define TYPE_NXT_SND 3              //!< Device is NXT sound sensorPort
#define NXT_SND_DB_MODE 0 
#define NXT_SND_DBA_MODE 1
#define TYPE_NXT_COL 4              //!< Device is NXT color sensor
#define NXT_COL_REFLECTED_MODE 0
#define NXT_COL_AMBIENT_MODE 1
#define NXT_COL_COLOR_MODE 2
#define NXT_COL_GREEN_MODE 3
#define NXT_COL_BLUE_MODE 4
#define NXT_COL_RAW_MODE 5
#define TYPE_NXT_US 5               //!< Device is NXT Ultrasonic sensor
#define NXT_US_CM_MODE 0
#define NXT_US_IN_MODE 1

// HT Sensors
#define TYPE_HT_DIR 52              //!< Device is HiTechnic Infrared Seeker
#define HT_DIR_DC_MODE 0
#define HT_DIR_AC_MODE 1

// Reserved device types
#define TYPE_NXT_IIC 123            //!< Device is NXT IIC sensor
#define TYPE_UNKNOWN 125            //!< Port not empty but type has not been determined
#define TYPE_NONE 126               //!< Port empty or not available
#define TYPE_ERROR 127              //!< Port not empty and type is invalid


#define MAX_DEVICE_TYPE 127         //!< Highest type number (positive)
#define MAX_DEVICE_MODES 8          //!< Max number of modes in one device
#define MAX_DEVICE_TYPES ((MAX_DEVICE_TYPE + 1) * MAX_DEVICE_MODES)//!< Max number of different device types and modes (max type data list size)

/***********************************/

int g_uartFile = 0;
int g_iicFile = 0;
int g_analogFile = 0;
int dcmFile = 0;
UART* g_uartSensors = 0;
IIC* g_iicSensors = 0;
ANALOG* g_analogSensors = 0;        // struct ANALOG defined in analog.h / lms2012.h 

static DEVCON devCon;
static IICDAT iicDat;
char buf[4];

int sensor_setup_NAME[INPUTS];
int ir_sensor_channel[INPUTS];

/********************************************************************************************/
/**
* Initialisation of the Sensorfunctions
* modified by: Simón Rodriguez Perez
* 		 date: 2015-02-28
*
*/
int InitSensors()
{
    g_uartFile = open("/dev/lms_uart", O_RDWR | O_SYNC);
    g_iicFile =  open("/dev/lms_iic", O_RDWR | O_SYNC);
    g_analogFile = open(ANALOG_DEVICE_NAME, O_RDWR | O_SYNC);
    dcmFile = open("/dev/lms_dcm", O_RDWR | O_SYNC);


    g_uartSensors = (UART*)mmap(0, sizeof(UART), PROT_READ | PROT_WRITE,
                                MAP_FILE | MAP_SHARED, g_uartFile, 0);
    g_iicSensors = (IIC*)mmap(0, sizeof(IIC), PROT_READ | PROT_WRITE,
                              MAP_FILE | MAP_SHARED, g_iicFile, 0);
    g_analogSensors = (ANALOG*)mmap(0, sizeof(ANALOG), PROT_READ | PROT_WRITE,
                                    MAP_FILE | MAP_SHARED, g_analogFile, 0);


	int i;
	for (i = 0; i < INPUTS; i++)
	{
		ir_sensor_channel[i] = 0;
		
		// SETUP DRIVERS
	    // Initialise pin setup string to do nothing
        buf[i] = '-';
        // build setup string for UART
        devCon.Connection[i] = g_analogSensors->InConn[i];
        devCon.Type[i] = g_analogSensors->InDcm[i];
        devCon.Mode[i] = 0;
        if (devCon.Connection[i] == CONN_INPUT_DUMB){
            devCon.Type[i] = 16;
        }
        if (devCon.Connection[i] == CONN_INPUT_UART){
            UARTCTL uartCtrl;
            uartCtrl.Port = i;
            uartCtrl.Mode = 0;
            ioctl(g_uartFile, UART_READ_MODE_INFO, &uartCtrl);
            devCon.Type[i] = uartCtrl.TypeData.Type;
            devCon.Mode[i] = uartCtrl.TypeData.Mode;
        }
        // TODO: if CONN_NXT_IIC
	}

	if (g_uartFile && g_iicFile && g_analogFile &&
		g_uartSensors && g_iicSensors && g_analogSensors)
		return 0;
	return -1;
}


/********************************************************************************************/
/**
* Getting the Data from a Uartport
* modified by: Simón Rodriguez Perez
* 		 date: 2015-02-28
* 		 note: Readout of Uart-Port
*
*/
void* readUartSensor(int sensorPort)
{
	if (!g_uartSensors)
		return 0;
    return g_uartSensors->Raw[sensorPort][g_uartSensors->Actual[sensorPort]];
}

void* readIicSensor(int sensorPort)
{
	if (!g_iicSensors)
		return 0;
    UWORD currentSensorSlot = g_iicSensors->Actual[sensorPort];
    //(unsigned char)pIic->Raw[IIC_PORT][pIic->Actual[IIC_PORT]][0]
    return g_iicSensors->Raw[sensorPort][currentSensorSlot];
}

void* readNewDumbSensor(int sensorPort)
{
	return (void*)&(g_analogSensors->InPin6[sensorPort]);
}

void* readOldDumbSensor(int sensorPort)
{
	return (void*)&(g_analogSensors->InPin1[sensorPort]);       // DATA16
}


/* stuff from c_input.c **********************************************/

#define   DATAF_NAN     ((float)0 / (float)0) //(0x7FC00000) from bytecodes.h l.1495

#define   FALSE                         0
#define   TRUE                          1

#define   SENSOR_RESOLUTION             1023L

/* Remember this is ARM AD converter  - 3,3 VDC as max voltage      */
/* When in color mode background value is substracted => min = 0!!! */
#define   AD_MAX                        2703L
#define   AD_FS                         3300L

#define   COLORSENSORBGMIN              (214/(AD_FS/AD_MAX))
#define   COLORSENSORMIN                (1L/(AD_FS/AD_MAX)) /* 1 inserted else div 0 (1L/(120/AD_MAX)) */
#define   COLORSENSORMAX                ((AD_MAX * AD_FS)/3300)
#define   COLORSENSORPCTDYN             (UBYTE)(((COLORSENSORMAX - COLORSENSORMIN) * 100L)/AD_MAX)
#define   COLORSENSORBGPCTDYN           (UBYTE)(((COLORSENSORMAX - COLORSENSORBGMIN) * 100L)/AD_MAX)

void      cInputCalcFullScale(UWORD *pRawVal, UWORD ZeroPointOffset, UBYTE PctFullScale, UBYTE InvStatus)
{
  if (*pRawVal >= ZeroPointOffset)
  {
    *pRawVal -= ZeroPointOffset;
  }
  else
  {
    *pRawVal = 0;
  }

  *pRawVal = (*pRawVal * 100)/PctFullScale;
  if (*pRawVal > SENSOR_RESOLUTION)
  {
    *pRawVal = SENSOR_RESOLUTION;
  }
  if (TRUE == InvStatus)
  {
    *pRawVal = SENSOR_RESOLUTION - *pRawVal;
  }
}

void cInputCalibrateColor(COLORSTRUCT *pC, UWORD *pNewVals)
{

  UBYTE CalRange;

  if ((pC->ADRaw[BLANK]) < pC->CalLimits[1])
  {
    CalRange = 2;
  }
  else
  {
    if ((pC->ADRaw[BLANK]) < pC->CalLimits[0])
    {
      CalRange = 1;
    }
    else
    {
      CalRange = 0;
    }
  }

  pNewVals[RED] = 0;
  if ((pC->ADRaw[RED]) > (pC->ADRaw[BLANK]))
  {
    pNewVals[RED] = (UWORD)(((ULONG)((pC->ADRaw[RED]) - (pC->ADRaw[BLANK])) * (pC->Calibration[CalRange][RED])) >> 16);
  }

  pNewVals[GREEN] = 0;
  if ((pC->ADRaw[GREEN]) > (pC->ADRaw[BLANK]))
  {
     pNewVals[GREEN] = (UWORD)(((ULONG)((pC->ADRaw[GREEN]) - (pC->ADRaw[BLANK])) * (pC->Calibration[CalRange][GREEN])) >> 16);
  }

  pNewVals[BLUE] = 0;
  if ((pC->ADRaw[BLUE]) > (pC->ADRaw[BLANK]))
  {
    pNewVals[BLUE] = (UWORD)(((ULONG)((pC->ADRaw[BLUE]) -(pC->ADRaw[BLANK])) * (pC->Calibration[CalRange][BLUE])) >> 16);
  }

  pNewVals[BLANK] = (pC->ADRaw[BLANK]);
  cInputCalcFullScale(&(pNewVals[BLANK]), COLORSENSORBGMIN, COLORSENSORBGPCTDYN, FALSE);
  (pNewVals[BLANK]) = (UWORD)(((ULONG)(pNewVals[BLANK]) * (pC->Calibration[CalRange][BLANK])) >> 16);
}

DATAF     cInputCalculateColor(COLORSTRUCT *pC)
{
  DATAF   Result ;


  Result  =  DATAF_NAN;

  // Color Sensor values has been calculated -
  // now calculate the color and put it in Sensor value
  

  if (((pC->SensorRaw[RED]) > (pC->SensorRaw[BLUE] )) &&
      ((pC->SensorRaw[RED]) > (pC->SensorRaw[GREEN])))
  {

    // If all 3 colors are less than 65 OR (Less that 110 and bg less than 40)
    if (((pC->SensorRaw[RED])   < 65) ||
        (((pC->SensorRaw[BLANK]) < 40) && ((pC->SensorRaw[RED])  < 110)))
    {
      Result  =  (DATAF)BLACKCOLOR;
    }
    else
    {
      if (((((pC->SensorRaw[BLUE]) >> 2)  + ((pC->SensorRaw[BLUE]) >> 3) + (pC->SensorRaw[BLUE])) < (pC->SensorRaw[GREEN])) &&
          ((((pC->SensorRaw[GREEN]) << 1)) > (pC->SensorRaw[RED])))
      {
        Result  =  (DATAF)YELLOWCOLOR;
      }
      else
      {

        if ((((pC->SensorRaw[GREEN]) << 1) - ((pC->SensorRaw[GREEN]) >> 2)) < (pC->SensorRaw[RED]))
        {

          Result  =  (DATAF)REDCOLOR;
        }
        else
        {

          if ((((pC->SensorRaw[BLUE]) < 70) ||
              ((pC->SensorRaw[GREEN]) < 70)) ||
             (((pC->SensorRaw[BLANK]) < 140) && ((pC->SensorRaw[RED]) < 140)))
          {
            Result  =  (DATAF)BLACKCOLOR;
          }
          else
          {
            Result  =  (DATAF)WHITECOLOR;
          }
        }
      }
    }
  }
  else
  {

    // Red is not the dominant color
    if ((pC->SensorRaw[GREEN]) > (pC->SensorRaw[BLUE]))
    {

      // Green is the dominant color
      // If all 3 colors are less than 40 OR (Less that 70 and bg less than 20)
      if (((pC->SensorRaw[GREEN])  < 40) ||
          (((pC->SensorRaw[BLANK]) < 30) && ((pC->SensorRaw[GREEN])  < 70)))
      {
        Result  =  (DATAF)BLACKCOLOR;
      }
      else
      {
        if ((((pC->SensorRaw[BLUE]) << 1)) < (pC->SensorRaw[RED]))
        {
          Result  =  (DATAF)YELLOWCOLOR;
        }
        else
        {
          if ((((pC->SensorRaw[RED]) + ((pC->SensorRaw[RED])>>2)) < (pC->SensorRaw[GREEN])) ||
              (((pC->SensorRaw[BLUE]) + ((pC->SensorRaw[BLUE])>>2)) < (pC->SensorRaw[GREEN])))
          {
            Result  =  (DATAF)GREENCOLOR;
          }
          else
          {
            if ((((pC->SensorRaw[RED]) < 70) ||
                ((pC->SensorRaw[BLUE]) < 70)) ||
                (((pC->SensorRaw[BLANK]) < 140) && ((pC->SensorRaw[GREEN]) < 140)))
            {
              Result  =  (DATAF)BLACKCOLOR;
            }
            else
            {
              Result  =  (DATAF)WHITECOLOR;
            }
          }
        }
      }
    }
    else
    {

      // Blue is the most dominant color
      // Colors can be blue, white or black
      // If all 3 colors are less than 48 OR (Less that 85 and bg less than 25)
      if (((pC->SensorRaw[BLUE])   < 48) ||
          (((pC->SensorRaw[BLANK]) < 25) && ((pC->SensorRaw[BLUE])  < 85)))
      {
        Result  =  (DATAF)BLACKCOLOR;
      }
      else
      {
        if ((((((pC->SensorRaw[RED]) * 48) >> 5) < (pC->SensorRaw[BLUE])) &&
            ((((pC->SensorRaw[GREEN]) * 48) >> 5) < (pC->SensorRaw[BLUE])))
            ||
            (((((pC->SensorRaw[RED])   * 58) >> 5) < (pC->SensorRaw[BLUE])) ||
             ((((pC->SensorRaw[GREEN]) * 58) >> 5) < (pC->SensorRaw[BLUE]))))
        {
          Result  =  (DATAF)BLUECOLOR;
        }
        else
        {

          // Color is white or Black
          if ((((pC->SensorRaw[RED])  < 60) ||
              ((pC->SensorRaw[GREEN]) < 60)) ||
             (((pC->SensorRaw[BLANK]) < 110) && ((pC->SensorRaw[BLUE]) < 120)))
          {
            Result  =  (DATAF)BLACKCOLOR;
          }
          else
          {
            if ((((pC->SensorRaw[RED])  + ((pC->SensorRaw[RED])   >> 3)) < (pC->SensorRaw[BLUE])) ||
                (((pC->SensorRaw[GREEN]) + ((pC->SensorRaw[GREEN]) >> 3)) < (pC->SensorRaw[BLUE])))
            {
              Result  =  (DATAF)BLUECOLOR;
            }
            else
            {
              Result  =  (DATAF)WHITECOLOR;
            }
          }
        }
      }
    }
  }


  return (Result);
}

/* end stuff c_input.c ***********************************************/

uint32_t readNxtColor(int sensorPort)
{
    uint32_t color = 0;
    cInputCalibrateColor(&g_analogSensors->NxtCol[sensorPort], g_analogSensors->NxtCol[sensorPort].SensorRaw);
    color = (uint32_t)cInputCalculateColor(&g_analogSensors->NxtCol[sensorPort]);
    return color;
}


/********************************************************************************************/
/**
* Get the Data from the Sensor
* modified by: Simón Rodriguez Perez
* 		 date: 2015-02-28
* 		 note: changed for Touch-, Sonar- and Lightsensor
*
*----------------------------------------------------------------------------------
*
* modified by: Simón Rodriguez Perez
* 		 date: 2016-04-21
* 		 note: readout for Gyro and Infrared Sensor
*
*/
void* ReadSensorData(int sensorPort)
{
	if (!g_analogSensors || sensorPort < 0 || sensorPort >= INPUTS)
		return 0;


	switch (sensor_setup_NAME[sensorPort])
	{
		case CONN_NONE: 
		case CONN_ERROR: 
		case NO_SEN: 
			return 0;
		// EV3 Touchsensor
		case TOUCH:
			return readNewDumbSensor(sensorPort);
		case BUMPS:
		    return readNewDumbSensor(sensorPort);
		// EV3 Colorsensor
		case COL_REFLECT: 
			return readUartSensor(sensorPort);
		case COL_AMBIENT: 
			return readUartSensor(sensorPort);
		case COL_COLOR: 
			return readUartSensor(sensorPort);
		// EV3 Ultrasonic
		case US_DIST_CM: 
			return readUartSensor(sensorPort);
		case US_DIST_MM: 
			return readUartSensor(sensorPort);
		case US_DIST_IN: 
			return readUartSensor(sensorPort);
		// EV3 Gyroskop
		case GYRO_ANG: 
			return readUartSensor(sensorPort);
		case GYRO_RATE: 
			return readUartSensor(sensorPort);
		// EV3 Infrared
		case IR_PROX:
		    return readUartSensor(sensorPort);
		case IR_SEEK:
		    return readUartSensor(sensorPort);
		case IR_REMOTE:
			return readUartSensor(sensorPort);
		// NXT
		case NXT_IR_SEEKER:
			return readIicSensor(sensorPort);
		case NXT_TEMP_C:
			return readIicSensor(sensorPort);
		case NXT_TEMP_F:
			return readIicSensor(sensorPort);
        case NXT_SND_DB:
            return readOldDumbSensor(sensorPort);
        case NXT_SND_DBA:
            return readOldDumbSensor(sensorPort);
        case NXT_TOUCH:
            return readOldDumbSensor(sensorPort);
        case NXT_REFLECT:
            return readOldDumbSensor(sensorPort);
        case NXT_AMBIENT:
            return readOldDumbSensor(sensorPort);
        case NXT_COL_REF:
            cInputCalibrateColor(&g_analogSensors->NxtCol[sensorPort], g_analogSensors->NxtCol[sensorPort].SensorRaw);
            return (void*)&g_analogSensors->NxtCol[sensorPort].ADRaw[RED];
        case NXT_COL_AMB:
            cInputCalibrateColor(&g_analogSensors->NxtCol[sensorPort], g_analogSensors->NxtCol[sensorPort].SensorRaw);
            return (void*)&g_analogSensors->NxtCol[sensorPort].ADRaw[BLANK];
        case NXT_COL_COL:
            return (void*)readNxtColor(sensorPort);
        case NXT_US_CM:
            return readIicSensor(sensorPort);
        case NXT_US_IN:
            return readIicSensor(sensorPort);
        // HiTechnic
        case HT_DIR_DC:
            return readIicSensor(sensorPort);
        case HT_DIR_AC:
            return readIicSensor(sensorPort);
        default: return 0;
    }

    return 0;
}

/********************************************************************************************/
/**
* Usercall for actual value of one Sensor
* modified by: Simón Rodriguez Perez
* 		 date: 2015-02-28
* 		 note: now working for Touch-, Sonar- and Lightsensor
*			   with calculation of the correct values
*
*----------------------------------------------------------------------------------
*
* modified by: Simón Rodriguez Perez
* 		 date: 2016-04-21
* 		 note: readout for Gyroscop and Infrared Sensor
* modified by: Bernd Sellentin
*        date: 2017-01
*        note: more NXT-sensors
*/
int ReadSensor(int sensorPort)
{
	uint64_t* data = ReadSensorData(sensorPort);
	
	int32_t help=0;
	int32_t value = 0;
	
	if (!data)
		return -1;

	switch (sensor_setup_NAME[sensorPort])
	{
		case NO_SEN:
			return -1;
		// Touchsensor
		case TOUCH:
			value = *((DATA16*)data);
			help = (value > 250) ? 1 : 0;
		    return help;
		case BUMPS:
		    // mentionned in ev3 firmware docs, but not used in EV-G, lejos, ev3-dev, ...
		    help = *((DATA16*)data);
			//help = help/256;
			return help;
		// Lightsensor
		case COL_REFLECT:
		    /* 
		     *  RawMax = 100, RawMin = 0
		     */
			return *((DATA16*)data)&0x00FF;
		case COL_AMBIENT:
		    /* 
		     *  RawMax = 100, RawMin = 0
		     */
			return *((DATA16*)data)&0x00FF;
		case COL_COLOR:
			/* RawMax = 8, RawMin = 0
			 * COLOR_NONE = 0
             * COLOR_BLACK = 1
             * COLOR_BLUE = 2
             * COLOR_GREEN = 3
             * COLOR_YELLOW = 4
             * COLOR_RED = 5
             * COLOR_WHITE = 6
             * COLOR_BROWN = 7
             */
			return *((DATA16*)data)&0x000F;
			
		// Ultrasonic
		case US_DIST_CM:
		    /* 
		     *  RawMax = 2550, RawMin = 0
		     */
			return (*((DATA16*)data)&0x0FFF)/10;
		case US_DIST_MM:
			return *((DATA16*)data)&0x0FFF;
		case US_DIST_IN:
		    /* 
		     *  RawMax = 1000, RawMin = 0
		     */
			return *((DATA16*)data)&0x0FFF;
		// Gyroskop
		case GYRO_ANG:
		    help = *((DATA16*)data)&0xFFFF;         // 1111 1111 1111 1111
		    if(help & 0x8000)                       // 1000 0000 0000 0000
			{
				help = ((help&0x7FFF) - 0x7FFF);    // 0111 1111 1111 1111
			}
			return help;
		case GYRO_RATE:
			help = *(data)&0xFFFF;
			if(help & 0x8000)
			{
				help = ((help&0x7FFF) - 0x7FFF);
			}
			return help;
		// Infrared
		case IR_PROX:
			return *((DATA16*)data)&0x00FF;
		case IR_SEEK:
			help = (*(data) >> (16*ir_sensor_channel[sensorPort]))& 0xFF;
			if(help & 0x80)
			{
				help = ((help&0x7F) - 0x7F);
			}
			return help;
		case IR_REMOTE:
			help = *(data)&0xFFFFFFFF;
			help = (help >> (8*ir_sensor_channel[sensorPort]))& 0xFF;
			return help;
		// NXT
		case NXT_IR_SEEKER:
			return *((DATA16*)data)&0x000F;
		case NXT_TEMP_C:
			help = (*data>>4) & 0x0FFF;
			if(help & 0x800)
			{
				help = ((help&0x7FF) ^ 0x7FF) + 1;
				return (-1)*(((help>>4) & 0xFF)*10 + ((help & 0xF) * 10 / 15));
			}
			return ((help>>4) & 0xFF)*10 + ((help & 0xF) * 10 / 15);
		case NXT_TEMP_F:
			help = (*data>>4) & 0x0FFF;
			if(help & 0x800)
			{
				help = ((help&0x7FF) ^ 0x7FF) + 1;
				return (-1)*(((help>>4) & 0xFF)*10 + ((help & 0xF) * 10 / 15)) * 9/5 + 320;
			}
			return (((help>>4) & 0xFF)*10 + ((help & 0xF) * 10 / 15)) * 9/5 + 320;
		case NXT_SND_DB:
		    /* 
		     *  RawMax = 0, RawMin = 4095
		     */ 
		    help = *((DATA16*)data)&0x0FFF;
		    return (100 - (help * 100/4095));
		    //return help;
		case NXT_SND_DBA:
		    /* 
		     *  RawMax = 0, RawMin = 4095
		     */ 
		    help = *((DATA16*)data)&0x0FFF;
		    return (100 - (help * 100/4095));
		case NXT_TOUCH:
		    value = *((DATA16*)data);
		    help = (value < 3800) ? 1 : 0;
		    return help;
		case NXT_REFLECT:
		    /*
		     * RawMax = 445, RawMin = 3372
		     */
		    value = *((DATA16*)data);
		    help = (3372-value)*100/(3372-445);
		    return help;
		case NXT_AMBIENT:
		    /*
		     * RawMax = 663, RawMin = 3411
		     */
		    value = *((DATA16*)data);
		    help = (3411-value)*100/(3411-663);
		    return help;
		case NXT_COL_REF:
		    /*
		     * RawMax = 1500, RawMin = 200
		     * RawMax seems to less
		     */
		    value = *((DATA16*)data);
		    help = 100 - (1800-value)*100/(1800-200);
		    return help;
		case NXT_COL_AMB:
		    /*
		     * RawMax = 2900, RawMin = 200
		     */
		    value = *((DATA16*)data);
		    help = 100 - (2900-value)*100/(2900-200);
		    return help;
		case NXT_COL_COL:
		    //value = *((DATA16*)data)&0x000F;
		    //return *((DATA16*)data);
		    return readNxtColor(sensorPort);
		case NXT_US_CM:
		    /*
		     * RawMax = 255, RawMin = 0
		     * SiMax = 255, SiMin = 0
		     */
		    return *((DATA16*)data);
		case NXT_US_IN:
		    /*
		     * RawMax = 255, RawMin = 0
		     * SiMax = 100, SiMin = 0
		     */
		    return *((DATA16*)data);
		case HT_DIR_DC:
		    return *((DATA16*)data)&0x000F;
		case HT_DIR_AC:
		    return *((DATA16*)data)&0x000F;
		default: break;
	}
	return *((DATA16*)data);
}

int GetSensorMode(int sensorPort)
{
    int mode;
    UARTCTL uartCtrl;
    uartCtrl.Port = sensorPort;
    ioctl(g_uartFile, UART_READ_MODE_INFO, &uartCtrl);
    mode = uartCtrl.TypeData.Mode;

    return devCon.Mode[sensorPort];
}
int GetSensorType(int sensorPort)
{
    int type;
    
    switch (g_analogSensors->InConn[sensorPort])
    {
        case CONN_NXT_COLOR :
            type = g_analogSensors->InDcm[sensorPort];
            break;
        case CONN_NXT_DUMB :
            type = g_analogSensors->InDcm[sensorPort];
            break;
        case CONN_NXT_IIC :
            /* this gives type 123 = TYPE_NXT_IIC
            type = g_analogSensors->InDcm[sensorPort];
            */
            {IICSTR iicStr;
            iicStr.Port = sensorPort;
            iicStr.Mode = 0;
            ioctl(g_iicFile, IIC_READ_TYPE_INFO, &iicStr);  // returns Manufacturer + SensorType
            /* see c_input.c l.2958
            while(Index < IIcDeviceTypes){
                if (strcmp((char*)iicStr.Manufacturer,(char*)iicStr[Index].Manufacturer)==0)
                {// Manufacturer found
                    if (strcmp((char*)iicStr.SensorType, (char*)iicStr[Index].SensorType)==0)
                    {// Type found
                        type = iicStr[Index].Type
                    }
                }
            }
            */
            type = iicStr.Type;} // still 0
            break;
        case CONN_INPUT_DUMB :
            type = 16;
            break;
        case CONN_INPUT_UART :
            {UARTCTL uartCtrl;
            uartCtrl.Port = sensorPort;
            uartCtrl.Mode = 0;
            ioctl(g_uartFile, UART_READ_MODE_INFO, &uartCtrl);
            type = uartCtrl.TypeData.Type;}
            break;
        case CONN_NONE:
            type = TYPE_UNKNOWN;
            break;
        default:
            type = 0;
    }
    
    return type;
}
/********************************************************************************************/
/**
* Initialisation for one Sensor 
* modified by: Simón Rodriguez Perez
* 		 date: 2015-02-28
* 		 note: Sensors are working now, but only one sensor is working at once
* modified by: Bernd Sellentin
         date: 2017-04
         note: works with multiple sensors
*
*/
int SetSensorMode(int sensorPort, int name)
{
	// data from http://ev3.fantastic.computer/doxygen/types.html
    int status;
    int i;
    int pins;
    //char buf[4];
    
	if (!g_analogSensors || sensorPort < 0 || sensorPort >= INPUTS)
		return -1;

	sensor_setup_NAME[sensorPort] = name;
	// Setup of Input
	switch (name)
	{
		case NO_SEN:
			break;
		// EV3-Touchsensor
		case TOUCH:
			devCon.Connection[sensorPort]	= CONN_INPUT_DUMB;      // in ev3_constants.h
			devCon.Type[sensorPort] 		= TOUCH_TYPE;           // here on top
			devCon.Mode[sensorPort] 		= TOUCH_PRESS_MODE;     // here on top
			pins = 0x46;
			break;
		case BUMPS:
		    devCon.Connection[sensorPort]	= CONN_INPUT_DUMB;
			devCon.Type[sensorPort] 		= TOUCH_TYPE;
			devCon.Mode[sensorPort] 		= TOUCH_BUMP_MODE;
			pins = 0x46;
			break;
		// EV3-Lightsensor
		case COL_REFLECT:
			devCon.Connection[sensorPort]	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= COL_TYPE;
			devCon.Mode[sensorPort] 		= COL_REFLECT_MODE;
			pins = 0x2D;
			break;
		case COL_AMBIENT:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= COL_TYPE;
			devCon.Mode[sensorPort] 		= COL_AMBIENT_MODE;
			pins = 0x2D;
			break;
		case COL_COLOR:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= COL_TYPE;
			devCon.Mode[sensorPort] 		= COL_COLOR_MODE;
			pins = 0x2D;
			break;
		// EV3-Ultrasonic
		case US_DIST_CM:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= US_TYPE;
			devCon.Mode[sensorPort] 		= US_DIST_CM_MODE;
			pins = 0x2D;
			break;
		case US_DIST_MM:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= US_TYPE;
			devCon.Mode[sensorPort] 		= US_DIST_MM_MODE;
			pins = 0x2D;
			break;
		case US_DIST_IN:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= US_TYPE;
			devCon.Mode[sensorPort] 		= US_DIST_IN_MODE;
			pins = 0x2D;
			break;
	    // EV3-Gyroskop
		case GYRO_ANG:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= GYRO_TYPE;
			devCon.Mode[sensorPort] 		= GYRO_ANG_MODE;
			pins = 0x2D;
			break;
		case GYRO_RATE:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= GYRO_TYPE;
			devCon.Mode[sensorPort] 		= GYRO_RATE_MODE;
			pins = 0x2D;
			break;
		// EV3-Infrared
		case IR_PROX:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= IR_TYPE;
			devCon.Mode[sensorPort] 		= IR_PROX_MODE;
			pins = 0x2D;
			break;
		case IR_SEEK:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= IR_TYPE;
			devCon.Mode[sensorPort] 		= IR_SEEK_MODE;
			pins = 0x2D;
			break;
		case IR_REMOTE:
			devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= IR_TYPE;
			devCon.Mode[sensorPort] 		= IR_REMOTE_MODE;
			pins = 0x2D;
			break;
		// NXT
		case NXT_IR_SEEKER:
			devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
			devCon.Type[sensorPort] 		= IIC_TYPE;
			devCon.Mode[sensorPort] 		= IIC_BYTE_MODE;
			pins = 0x46;
			iicDat.Port = sensorPort;
		    iicDat.Time = 0;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 1;               // Direction data = 1 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x01;        // I2C address
		    iicDat.WrData[1] = 0x42;        // register to read
			break;
		case NXT_TEMP_C:
			devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
			devCon.Type[sensorPort] 		= NXT_TEMP_TYPE;
			devCon.Mode[sensorPort] 		= NXT_TEMP_C_MODE;
			pins = 0x46;
			iicDat.Port = sensorPort;
		    iicDat.Time = 300;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 2;               // data = 2 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x4C;        // I2C address
		    iicDat.WrData[1] = 0x00;        // register to read
			break;
		case NXT_TEMP_F:
			devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
			devCon.Type[sensorPort] 		= NXT_TEMP_TYPE;
			devCon.Mode[sensorPort] 		= NXT_TEMP_F_MODE;
			pins = 0x46;
			iicDat.Port = sensorPort;
		    iicDat.Time = 300;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 2;               // data = 2 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x4C;        // I2C address
		    iicDat.WrData[1] = 0x00;        // register to read
			break;
		case NXT_SND_DB:
	        devCon.Connection[sensorPort] 	= CONN_NXT_DUMB;
			devCon.Type[sensorPort] 		= TYPE_NXT_SND;
			devCon.Mode[sensorPort] 		= NXT_SND_DB_MODE;
			pins = 0x30;
			break;
	    case NXT_SND_DBA:
	        devCon.Connection[sensorPort] 	= CONN_NXT_DUMB;
			devCon.Type[sensorPort] 		= TYPE_NXT_SND;
			devCon.Mode[sensorPort] 		= NXT_SND_DBA_MODE;
			pins = 0x32;
			break;
		case NXT_TOUCH:
		    devCon.Connection[sensorPort] 	= CONN_NXT_DUMB;
		    devCon.Type[sensorPort] 		= TYPE_NXT_TOUCH;
		    devCon.Mode[sensorPort] 		= NXT_TOUCH_MODE;
		    pins = 0x46;
		    break;
		case NXT_REFLECT:
		    devCon.Connection[sensorPort] 	= CONN_NXT_DUMB;
		    devCon.Type[sensorPort] 		= TYPE_NXT_LIGHT;
		    devCon.Mode[sensorPort] 		= NXT_LIGHT_REFLECTED_MODE;
		    pins = 0x32;
		    break;
		case NXT_AMBIENT:
		    devCon.Connection[sensorPort] 	= CONN_NXT_DUMB;
		    devCon.Type[sensorPort] 		= TYPE_NXT_LIGHT;
		    devCon.Mode[sensorPort] 		= NXT_LIGHT_AMBIENT_MODE;
		    pins = 0x30;
		    break;
		case NXT_COL_REFLECTED_MODE:
		    devCon.Connection[sensorPort] 	= CONN_NXT_COLOR;
		    devCon.Type[sensorPort] 		= TYPE_NXT_COL;
		    devCon.Mode[sensorPort] 		= NXT_COL_REFLECTED_MODE;
		    pins = 0x0E;
		    break;
		case NXT_COL_AMB:
		    devCon.Connection[sensorPort] 	= CONN_NXT_COLOR;
		    devCon.Type[sensorPort] 		= TYPE_NXT_COL;
		    devCon.Mode[sensorPort] 		= NXT_COL_AMBIENT_MODE;
		    pins = 0x11;
		    break;
		case NXT_COL_COL:
		    devCon.Connection[sensorPort] 	= CONN_NXT_COLOR;
		    devCon.Type[sensorPort] 		= TYPE_NXT_COL;
		    devCon.Mode[sensorPort] 		= NXT_COL_COLOR_MODE;
		    pins = 0x0D;
		    break;
		case NXT_US_CM:
		    devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
		    devCon.Type[sensorPort] 		= TYPE_NXT_US;
		    devCon.Mode[sensorPort] 		= NXT_US_CM_MODE;
		    pins = 0x31;
		    iicDat.Port = sensorPort;
		    iicDat.Time = 300;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 1;               // data = 1 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x01;        // I2C address
		    iicDat.WrData[1] = 0x42;        // register to read
		    break;
		case NXT_US_IN:
		    devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
		    devCon.Type[sensorPort] 		= TYPE_NXT_US;
		    devCon.Mode[sensorPort] 		= NXT_US_CM_MODE;
		    pins = 0x31;
		    iicDat.Port = sensorPort;
		    iicDat.Time = 300;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 1;               // data = 1 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x01;        // I2C address
		    iicDat.WrData[1] = 0x42;        // register to read
		    break;
		// HiTechnic Infrared Seeker
		case HT_DIR_DC:
		    devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
		    devCon.Type[sensorPort]         = TYPE_HT_DIR;
		    devCon.Mode[sensorPort]         = HT_DIR_DC_MODE;
		    pins = 0x46;
		    iicDat.Port = sensorPort;
		    iicDat.Time = 100;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 1;               // Direction data = 1 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x10;        // I2C address
		    iicDat.WrData[1] = 0x42;        // register to read
		    break;
		case HT_DIR_AC:
		    devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
		    devCon.Type[sensorPort]         = TYPE_HT_DIR;
		    devCon.Mode[sensorPort]         = HT_DIR_DC_MODE;
		    pins = 0x46;
		    iicDat.Port = sensorPort;
		    iicDat.Time = 100;
		    iicDat.Repeat = 0;
		    iicDat.RdLng = 1;               // Direction data = 1 byte
		    iicDat.WrLng = 2;
		    iicDat.WrData[0] = 0x10;        // I2C address
		    iicDat.WrData[1] = 0x49;        // register to read
		    break;
		default: return -1;
	}
	
	g_analogSensors->Updated[sensorPort]    =  0;
	buf[sensorPort] = pins;
    // write setup string to "Device Connection Manager" driver
    write(dcmFile, buf, 4); // necessary for analog devices
	
	if (devCon.Connection[sensorPort] == CONN_INPUT_UART){
	    g_uartSensors->Status[sensorPort]      &= ~UART_DATA_READY;
	
        UARTCTL uartCtrl;
        uartCtrl.Port = sensorPort;
        uartCtrl.Mode = devCon.Mode[sensorPort];
	    // write setup string to "UART Device Controller" driver
        ioctl(g_uartFile, UART_SET_CONN, &devCon);
        
	    status = wait_no_zero_status(sensorPort);
	    if (status & UART_PORT_CHANGED){
	        // Clear the port changed flag for the current port.
	        ioctl(g_uartFile, UART_CLEAR_CHANGED, &uartCtrl);
	        g_uartSensors->Status[sensorPort] &= ~UART_PORT_CHANGED;
	    }
	}

	if(devCon.Connection[sensorPort] == CONN_NXT_IIC){
	    g_iicSensors->Status[sensorPort] &= ~IIC_DATA_READY;
	    
	    IICCTL iicCtrl;
	    iicCtrl.Port = sensorPort;
	    iicCtrl.Mode = devCon.Mode[sensorPort];
	    ioctl(g_iicFile, IIC_SET_CONN, &devCon);
	    
	    /*int i, status;
	    for(i=0;i<50;i++){
	        status = g_iicSensors->Status[sensorPort];
	        if(status != 0){
	            break;
	        }
	        sleep(0.1);
	    }
	    if(status && IIC_PORT_CHANGED){
	    
	    }*/
	}

	return 0;
}

int wait_no_zero_status(int sensorPort)
{
    int status;
    int i=0;
    
    for(i=0;i<50;i++){
        status = g_uartSensors->Status[sensorPort];
        if(status != 0){
            break;
        }
        sleep(0.1);
    }
    return status;
}

int clear_change(int sensorPort)
{
    //static DEVCON devCon;
    int status;
    int i=0;
    for(i=0;i<50;i++){
        status = g_uartSensors->Status[sensorPort];
        if ((status & UART_DATA_READY != 0) && (status & UART_PORT_CHANGED) == 0){
            break;
        }
        devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
		devCon.Type[sensorPort] 		= 0;
		devCon.Mode[sensorPort] 		= 0;
		ioctl(g_uartFile, UART_CLEAR_CHANGED, &devCon);
		sleep(0.1);
    }
}

/** Reset the angle of the gyrosensor to 0 by changing modes back and forth
 *
 */
int ResetGyro()
{   
    //static DEVCON devCon;
    int sensorPort = 0;
    int mode = 0;
    DATA8 type = 0;
    int resulta;
    int resultb;
    
    sleep(1);
    
    for(sensorPort=0; sensorPort<4; sensorPort++)
    {
        // switch to other mode
        if(sensor_setup_NAME[sensorPort] == GYRO_RATE)      // 9
        {
            //return sensorPort;  //    works
            //type = *(g_uartSensors->Raw[sensorPort][g_uartSensors->TypeData[sensorPort][1].Type]); //works not: incompatible types in return
            // g_uartSensors->Raw[sensorPort][g_uartSensors->Actual[sensorPort]];
            
            mode = GYRO_RATE;
            devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= GYRO_TYPE;        // 32
			devCon.Mode[sensorPort] 		= GYRO_ANG_MODE;    // 3
			ioctl(g_uartFile, UART_SET_CONN, &devCon);
            break;
        }
        else if(sensor_setup_NAME[sensorPort] == GYRO_ANG)    // 8
        {
            mode = GYRO_ANG;
            devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
			devCon.Type[sensorPort] 		= GYRO_TYPE;        // 32
			devCon.Mode[sensorPort] 		= GYRO_RATE_MODE;   // 3
			ioctl(g_uartFile, UART_SET_CONN, &devCon);
			break;
        }
    }
    
    while(!resulta){
        resulta = ReadSensor(sensorPort);
    }
    
    sleep(1);
    
    switch (mode)       // switch to old mode
    {
        case GYRO_RATE:
            devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
	        devCon.Type[sensorPort] 		= GYRO_TYPE;
	        devCon.Mode[sensorPort] 		= GYRO_RATE_MODE;
	        ioctl(g_uartFile, UART_SET_CONN, &devCon);
	        break;
        case GYRO_ANG:
            devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
	        devCon.Type[sensorPort] 		= GYRO_TYPE;
	        devCon.Mode[sensorPort] 		= GYRO_ANG_MODE;
	        ioctl(g_uartFile, UART_SET_CONN, &devCon);
	        break;
	    default:
	        return -1;
    }
    
    while(!resultb){
        resultb = ReadSensor(sensorPort);
    }
    sleep(1);
    return sensorPort+1;
}

/********************************************************************************************/
/**
* Initialisation of all Sensors for working parallel 
* author: Simón Rodriguez Perez
* note: the function can only be called once at the beginning
*
*/
int SetAllSensorMode(int name_1, int name_2, int name_3, int name_4)
{
	//static DEVCON devCon;
	int sensorPort = 0;
	
	int name[4] = {};

	name[0] = name_1;
	name[1] = name_2;
	name[2] = name_3;
	name[3] = name_4;

	if (!g_analogSensors)
	{
		return -1;
	}
	
	// Setup of Input
	for(sensorPort=0; sensorPort<4; sensorPort++)
	{
		sensor_setup_NAME[sensorPort] = name[sensorPort];
		switch (name[sensorPort])
		{
			case NO_SEN:
				break;
			// Touchsensor
			case TOUCH:
				devCon.Connection[sensorPort] 	= CONN_INPUT_DUMB;
				devCon.Type[sensorPort] 		= TOUCH_TYPE;
				devCon.Mode[sensorPort] 		= TOUCH_PRESS_MODE;
				break;
			case BUMPS:
			    devCon.Connection[sensorPort] 	= CONN_INPUT_DUMB;
				devCon.Type[sensorPort] 		= TOUCH_TYPE;
				devCon.Mode[sensorPort] 		= TOUCH_BUMP_MODE;
				break;
			// Lightsensor
			case COL_REFLECT:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= COL_TYPE;
				devCon.Mode[sensorPort] 		= COL_REFLECT_MODE;
				break;
			case COL_AMBIENT:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= COL_TYPE;
				devCon.Mode[sensorPort] 		= COL_AMBIENT_MODE;
				break;
			case COL_COLOR:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= COL_TYPE;
				devCon.Mode[sensorPort] 		= COL_COLOR_MODE;
				break;
			// Ultrasonic
			case US_DIST_CM:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= US_TYPE;
				devCon.Mode[sensorPort] 		= US_DIST_CM_MODE;
				break;
			case US_DIST_MM:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= US_TYPE;
				devCon.Mode[sensorPort] 		= US_DIST_MM_MODE;
				break;
			case US_DIST_IN:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= US_TYPE;
				devCon.Mode[sensorPort] 		= US_DIST_IN_MODE;
				break;
			// Gyroskop
			case GYRO_ANG:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= GYRO_TYPE;
				devCon.Mode[sensorPort] 		= GYRO_ANG_MODE;
				break;
			case GYRO_RATE:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= GYRO_TYPE;
				devCon.Mode[sensorPort] 		= GYRO_RATE_MODE;
				break;
			// Infrared
			case IR_PROX:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= IR_TYPE;
				devCon.Mode[sensorPort] 		= IR_PROX_MODE;
				break;
			case IR_SEEK:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= IR_TYPE;
				devCon.Mode[sensorPort] 		= IR_SEEK_MODE;
				break;
			case IR_REMOTE:
				devCon.Connection[sensorPort] 	= CONN_INPUT_UART;
				devCon.Type[sensorPort] 		= IR_TYPE;
				devCon.Mode[sensorPort] 		= IR_REMOTE_MODE;
				break;
			// NXT
			case NXT_IR_SEEKER:
				devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
				devCon.Type[sensorPort] 		= IIC_TYPE;
				devCon.Mode[sensorPort] 		= IIC_BYTE_MODE;
				break;
			case NXT_TEMP_C:
				devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
				devCon.Type[sensorPort] 		= NXT_TEMP_TYPE;
				devCon.Mode[sensorPort] 		= NXT_TEMP_C_MODE;
				break;
			case NXT_TEMP_F:
				devCon.Connection[sensorPort] 	= CONN_NXT_IIC;
				devCon.Type[sensorPort] 		= NXT_TEMP_TYPE;
				devCon.Mode[sensorPort] 		= NXT_TEMP_F_MODE;
				break;
			default: return -1;
		}
	}
	// Set actual device mode
	ioctl(g_uartFile, UART_SET_CONN, &devCon);
	//ioctl(g_iicFile, IIC_SET_CONN, &devCon);
	return 0;
}


/********************************************************************************************/
/**
* Selection of the Beacon channel for IR Sensor
* author: Simón Rodriguez Perez
* note: channel can be modified while running
*
*/
int SetIRBeaconCH(int sensorPort, int channel)
{
	ir_sensor_channel[sensorPort] = channel;
	
	return 0;
}



