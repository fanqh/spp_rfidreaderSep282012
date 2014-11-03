#ifndef SPPB_H
#define SPPB_H

#include <message.h>
#include <spp.h>

#include "messagebase.h"


/* Persistent store base for device attributes */
#define SPPA_ATTR_PS_BASE		        (0)
#define SPPA_DEV_DEFAULT_ADDR_INDEX		(0)

/** **/
#define SPPB_PAIRABLE_DURATION 		(90000)



#if 0
/** command value **/
#define	UART_CMD_PAIR				(0x0200)
#define	UART_CMD_GOCONNECTABLE	    (0x0300)
#define	UART_CMD_DISCONNECT		    (0x0500)
#define	UART_CMD_QUITBT			    (0x0600)
#define	UART_CMD_SEND				(0x0700)

#define	K_Cmd_Len	        (2)
#define	K_DataHeader_Len  (12)
/** response value **/
#define	UART_RESP_FAILED					(0x0000)
#define	UART_RESP_READY_NOHOSTADD			(0x0100)
#define	UART_RESP_READY_WITHHOSTADD		    (0x0101)

#define	UART_RESP_PAIR_SUCCESS				(0x0201)
#define	UART_RESP_PAIR_TIMEOUT				(0x0202)

#define	UART_RESP_CONNECTABLE_SUCCESS		(0x0301)
#define	UART_RESP_CONNECTABLE_NOHOSTADD	    (0x0302)

#define	UART_RESP_CONNECTED_SUCCESS			(0x0401)
#define	UART_RESP_DISCONNECT_SUCCESS		(0x0501)
#define	UART_RESP_QUITBT_SUCCESS			(0x0601)

#define	UART_RESP_SEND_SUCCESS				(0x0701)
#define	UART_RESP_SEND_FAILED				(0x0702)
#define	UART_RESP_SEND_VERIFYFAILED			(0x0703)
#define	UART_RESP_HOST_LOST					(0x0801)
#endif





/** command value **/

#define	UART_CMD_INITDEVICEID		        (0x0011)
#define	UART_CMD_PAIR				        (0x0002)
#define	UART_CMD_GOCONNECTABLE	            (0x0003)
#define	UART_CMD_DISCONNECT		            (0x0005)
#define	UART_CMD_QUITBT			            (0x0006)
#define	UART_CMD_SEND				        (0x0007)
#define	UART_CMD_CONNECT			        (0x0009)


#define	K_Cmd_Len                    (2)
#define	K_Cmd_SendDataLength_Len     (2)
#define	K_DataHeader_Len             (12)
#define K_DEVICEID_LEN               (4)
#define K_MAX_DEVICEID_LEN           (32)

/** response value **/
#define	UART_RESP_FAILED					(0xffff)
#define	UART_RESP_READY_NOHOSTADD			(0x0001)
#define	UART_RESP_READY_WITHHOSTADD		    (0x0101)

#define	UART_RESP_READY_INITDEVICEID	    (0x0111)

#define	UART_RESP_PAIRCMD_RECIEVED			(0x0102)
#define	UART_RESP_PAIR_SUCCESS				(0x0302)
#define	UART_RESP_PAIR_TIMEOUT				(0x0202)

#define	UART_RESP_CONNECTABLE_SUCCESS		(0x0103)
#define	UART_RESP_CONNECTABLE_NOHOSTADD	    (0x0203)

#define	UART_RESP_CONNECTED_SUCCESS			(0x0104)

#define	UART_RESP_DISCONNECT_SUCCESS		(0x0105)

#define	UART_RESP_QUITBT_SUCCESS			(0x0106)

#define	UART_RESP_SEND_SUCCESS				(0x0107)
#define	UART_RESP_SEND_FAILED				(0x0207)
#define	UART_RESP_SEND_VERIFYFAILED			(0x0307)
#define	UART_RESP_HOST_LOST					(0x0108)
#define	UART_RESP_CONNECTBYSPPA_SUCCESS	    (0x0109)

/** command type **/
typedef enum
{
    CMD_INITDEVICEID,
	CMD_PAIR,
 	CMD_GOCONNECTABLE,
 	CMD_DISCONNECT,
 	CMD_QUITBT,
 	CMD_SEND,
 	CMD_CONNECT,
 	CMD_UNKOWN
 
} uart_cmd_type;



/** sppb state **/
typedef enum
{
	SPPB_INITIALISING,
	SPPB_READY,				/** this is the initialized and stable state **/
	SPPB_CONNECTAS_SPPA, 
	SPPB_DISCONNECTED,
	SPPB_PAIRABLE,
	SPPB_IDLE,
	SPPB_CONNECTED,
	SPPB_DISCONNECTING
	
} sppb_state_t;


/** sppb task data, noting that many of them are state- or substate-specific, do create/destroy in entry/exit funcs **/
typedef struct 
{
	/** task **/
    TaskData            task;
	
	/** hal task **/
	Task				hal_task;
	
	/** initialisation result **/	
	bool				cl_initialised;		
	bool				spp_initialised;
	
	/** scanning state locals **/
	bdaddr              bd_addr;
	bdaddr			    tempBd_addr;
	bool				connecting;
	bool 			    dataRecieving;
	uint16			    dataRecieveCounter;
	
	/** connected state-specific parameters 						**/
	SPP*			    spp;					/* connected state 	**/  /** for connected state parameters, don't clean up when transition between sub-state, 	**/
	Sink				spp_sink;				/* connected state 	**/  /** init and clean in connected enter/exit 											**/

	/** main state & sub state */
    sppb_state_t        state;
    
    uint32 deviceID;
	
} sppb_task_t;

void sppb_init(void);

Task getSppbTask(void);


#endif /** SPPB_H **/


