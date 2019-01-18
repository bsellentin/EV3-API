
#ifndef EV3_UART_H_
#define EV3_UART_H_

#include "ev3_basictypes.h"
#include "ev3_typedata.h"

/*! \page UartModuleMemory
 *
 *  <b>     Shared Memory </b>
 *
 *  <hr size="1"/>
 *
 *  It is possible to get a pointer to the uart values for use in userspace
 *  this pointer will point to a struct and the layout is following:
 *
 *  \verbatim
 */

#define    UART_DATA_LENGTH         MAX_DEVICE_DATALENGTH   // in ev3_constants
#define    UART_BUFFER_SIZE         64
#define    UART_PORT_CHANGED        0x01                    //!< Input port changed
#define    UART_DATA_READY          0x08                    //!< Data is ready
#define    UART_WRITE_REQUEST       0x10                    //!< Write request
#define    UART_SET_CONN            _IOWR('u',0,DEVCON)
#define    UART_READ_MODE_INFO      _IOWR('u',1,UARTCTL)
#define    UART_NACK_MODE_INFO      _IOWR('u',2,UARTCTL)
#define    UART_CLEAR_CHANGED       _IOWR('u',3,UARTCTL)

typedef   struct
{
  TYPES   TypeData[INPUTS][MAX_DEVICE_MODES]; //!< TypeData

#ifndef DISABLE_FAST_DATALOG_BUFFER
  uint16_t   Repeat[INPUTS][DEVICE_LOGBUF_SIZE];
  DATA8   Raw[INPUTS][DEVICE_LOGBUF_SIZE][UART_DATA_LENGTH];      //!< Raw value from UART device
  uint16_t   Actual[INPUTS];
  uint16_t   LogIn[INPUTS];
#else
  DATA8   Raw[INPUTS][UART_DATA_LENGTH];      //!< Raw value from UART device
#endif
  DATA8   Status[INPUTS];                     //!< Status
  DATA8   Output[INPUTS][UART_DATA_LENGTH];   //!< Bytes to UART device
  DATA8   OutputLength[INPUTS];
}
UART;

/*\endverbatim
 *
 *  \n
 */


typedef   struct
{
  TYPES   TypeData;
  DATA8   Port;
  DATA8   Mode;
}
UARTCTL;


#endif //EV3_UART_H_
