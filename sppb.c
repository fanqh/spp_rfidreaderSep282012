/* Copyright (C) Cambridge Silicon Radio Limited 2005-2009 */
/* Part of BlueLab 4.1.2-Release */


#include <connection.h>
#include <panic.h>
#include <stdio.h>
#include <stream.h>
#include <pio.h>
#include <panic.h>
#include <sink.h>
#include <source.h>
#include <string.h>
#include <bdaddr.h>


#include "debug.h"
#include "sppb.h"
#include "spp_dev_private.h"
#include "spp_dev_auth.h"
#include "spp_dev_b_leds.h"
#include "spp_dev_b_buttons.h"
#include "hal.h"
#include "errman.h"



/** task data **/
static sppb_task_t sppb;

/**************************************************************************************************
  
  from spp_dev_init.h & .c

  */

#define K_DEVICENAME_MAX_LEN    14
#define K_MILLIN_SEC 5000
static void sppDevInit(void);

static void sppDevInit()
{
    spp_init_params init;

    init.client_recipe = 0;
    init.size_service_record = 0;
	init.service_record = 0;
	init.no_service_record = 0;
	
    /* Initialise the spp profile lib, stating that this is device B */ 
    SppInitLazy(getSppbTask(), getSppbTask(), &init);
}

#define CLASS_OF_DEVICE		0x1F00


/**************************************************************************************************
  
  sppb
  
  */

/** debug output **/
static void unhandledSppState(sppb_state_t state, MessageId id);


/** state entry / exit and handers **/
static void sppb_handler(Task task, MessageId id, Message message);

/** separate spp layer and cl layer **/
static void cl_handler(Task task, MessageId id, Message message);

/** main state handlers & entry/exits **/
static void sppb_initialising_state_handler(Task task, MessageId id, Message message);
static void sppb_ready_state_handler(Task task, MessageId id, Message message);
/*
static void sppb_connectAsSPPA_state_handler(Task task, MessageId id, Message message);
*/
static void sppb_disconnected_state_handler(Task taks, MessageId id, Message message);
static void sppb_pairable_state_handler(Task taks, MessageId id, Message message);
static void sppb_idle_state_handler(Task taks, MessageId id, Message message);
static void sppb_connected_state_handler(Task task, MessageId id, Message message);
static void sppb_disconnecting_state_handler(Task task, MessageId id, Message message);

static void sppb_initialising_state_enter(void);
static void sppb_initialising_state_exit(void);
static void sppb_ready_state_enter(void);
static void sppb_ready_state_exit(void);

static void sppb_connectAsSPPA_state_enter(void);
static void sppb_connectAsSPPA_state_exit(void);

static void sppb_disconnected_state_enter(void);		
static void sppb_disconnected_state_exit(void);
static void sppb_pairable_state_enter(void);		
static void sppb_pairable_state_exit(void);
static void sppb_idle_state_enter(void);		

static void sppb_connected_state_enter(void);
static void sppb_connected_state_exit(void);
static void sppb_disconnecting_state_enter(void);
static void sppb_disconnecting_state_exit(void);

/** Duty Classes **/
static bool ConnectHostAsSppA(void);
static const sppb_state_t getSppState(void);
static const bool haveHostBDAddr(void);
static void UpdateNewPairedDevice( const bdaddr* addr );

static bool sendRespToUart( uint16 resp );
static bool send(Sink sink, const uint8 * aData, uint16 len );
static uart_cmd_type processUartData( const uint8* aData, uint16 len );
static uart_cmd_type recieveDataFromUart(void);
static bool sendDataToSpp( const uint8* aData, uint16 datalen );
static uint16 getDataBodyLen( const uint8* aData );

/* Common */
static const uint32 bytesToUInt( const uint8* byte );


Task getSppbTask(void)
{
    return &sppb.task;
}

static const uint32 bytesToUInt( const uint8* byte )
{
    uint8 i;
    uint32 ret =0;
    for( i = 0;i < 4; i++ )
    {
        ret = ret << 8;
        ret += byte[3-i];
        
    }
    return ret;
}

static const sppb_state_t getSppState(void)
{
    return sppb.state;
}

static uart_cmd_type processUartData( const uint8* aData, uint16 len )
{
	uart_cmd_type cmdType;
	cmdType = CMD_UNKOWN;
	
	if( len >= K_Cmd_Len )
	{	
		uint16 cmd;
		cmd = aData[0]* 0x100 + aData[1] ;
/*		uint16* cmd;
		cmd = (uint16*)(aData);*/
		
		switch( cmd )
		{
			case (uint16)UART_CMD_INITDEVICEID:
                if( K_Cmd_Len + K_DEVICEID_LEN == len )
                {
                    /* set up a new deviceid from stm32*/
                    sppb.deviceID = bytesToUInt( (uint8*)&aData[K_Cmd_Len] );
			        DEBUG(( " recieve command which type is CMD_INITDEVICEID, value is %x.. \n", cmd ));	
			        cmdType = CMD_INITDEVICEID;
                }
				break;            
			case (uint16)UART_CMD_PAIR:
				DEBUG(( " recieve command which type is CMD_PAIR, value is %x.. \n", cmd ));	
				cmdType = CMD_PAIR;
				break;
			case UART_CMD_GOCONNECTABLE:
				DEBUG(( " recieve command which type is CMD_GOCONNECTABLE, value is %x.. \n", cmd ));	
				cmdType = CMD_GOCONNECTABLE;
				break;
			case UART_CMD_DISCONNECT:
				DEBUG(( " recieve command which type is CMD_DISCONNECT, value is %x.. \n", cmd ));	
				cmdType = CMD_DISCONNECT;
				break;
			case UART_CMD_QUITBT:
				DEBUG(( " recieve command which type is CMD_QUITBT, value is %x.. \n", cmd ));	
				cmdType = CMD_QUITBT;
				break;
			case UART_CMD_SEND:
				DEBUG(( " recieve command which type is CMD_SEND, value is %x.. \n", cmd ));	
				cmdType = CMD_SEND;
				break;
			case UART_CMD_CONNECT:
				DEBUG(( " recieve command which type is CMD_CONNECT, value is %x.. \n", cmd ));	
				cmdType = CMD_CONNECT;
				break;
				
			default:
				DEBUG(( " !!!! recieve command which type is bull Shit, value is %x.. \n", cmd ));
				break;
		}
	}
	return cmdType;
}


static bool sendRespToUart( uint16 resp )
{
	uint8 respArray[K_Cmd_Len];
	bool ret = FALSE;
    Sink uart = StreamUartSink();
	
	respArray[1] = (uint8)resp;
	respArray[0] = (uint8)(resp >> 8 );
	ret = send( uart, respArray , K_Cmd_Len );

	return ret;
	
}

static bool send(Sink sink, const uint8 * aData, uint16 len )
{
	bool ret;
	
	ret = FALSE;
	if(sink && SinkClaim(sink, len) != 0xFFFF)
	{
		memcpy(SinkMap(sink), aData, len );
		(void) PanicZero(SinkFlush(sink, len));

		ret =  TRUE;
	}
	return ret;
}

static uart_cmd_type recieveDataFromUart(void)
{
	uint16 i;
	Source source;
	uint16 size;
	uint16 lastSize;
	const uint8* ptr;
	uart_cmd_type cmdType;

	lastSize = 0;
	size = 0;
	cmdType = CMD_UNKOWN;
	source = StreamUartSource();
	
	do
	{
		lastSize = size;
		size = SourceSize(source);
		busy_wait(1);		
	}while(lastSize != size );
		
	ptr = SourceMap(source);	
	
	DEBUG(( "    source has %d bytes of data... \n", size ));
	for(i = 0; i< size; i++)
	{
		DEBUG(( " source data is %x .. \n", ptr[i] ));		
	}

	cmdType = processUartData( ptr, size);
			
	SourceDrop(source, size);

	return cmdType;
	
}

static bool sendDataToSpp( const uint8* aData, uint16 dataLen )
{
	bool ret = FALSE;
	if ( dataLen && aData && SinkIsValid(sppb.spp_sink) && (SinkSlack(sppb.spp_sink) >= dataLen) ) 
	{
		uint16 offset, packetLen;
		uint8* dst;
		Sink sink = sppb.spp_sink;

		/* swap high with low bytes for dataLen */
		packetLen = dataLen;

		offset = SinkClaim(sink, packetLen);
		dst = SinkMap(sink);
		memcpy(dst + offset, aData, packetLen );
		SinkFlush(sink, packetLen);
		
		DEBUG(( "    send %d bytes to spp correctly... \n", packetLen ));

		ret = TRUE;
	}
	return ret;
}

static uint16 getDataBodyLen( const uint8* aData )
{
	return 256+12 +2;
}



static void unhandledSppState(sppb_state_t state, MessageId id)
{
    DEBUG(("SPP current state %d message id 0x%x\n", state, id));   
}


static void setSppState(const sppb_state_t state)
{
    DEBUG(("SPP State - C=%d N=%d\n",sppb.state, state));
    sppb.state = state;
}


static void UpdateNewPairedDevice( const bdaddr* addr  )
{
	if( !BdaddrIsZero(addr) )
	{
		ConnectionSmSetTrustLevel( addr , TRUE);
		sppb.bd_addr = *addr;
	}
}

static bool HandleClSmGetIndexedAttributeConfirm( CL_SM_GET_INDEXED_ATTRIBUTE_CFM_T *cfm )
{
	bool ret = FALSE;
	DEBUG(("appHandleClSmGetIndexedAttributeConfirm\n"));
	
	if (cfm->status == success)
	{
		/* Store Bluetooth address of host */
		sppb.bd_addr = cfm->bd_addr;   
		ret = TRUE;
	}
	return ret;
}

static const bool haveHostBDAddr(void)
{
	return !BdaddrIsZero( &sppb.bd_addr) ;
}

static bool ConnectHostAsSppA(void)
{
	bool ret = FALSE;
	DEBUG(("ConnectHostAsSppA\n"));

	if ( SPPB_CONNECTAS_SPPA == getSppState() )
	{
		if ( FALSE == sppb.connecting ) 
		{
			if ( haveHostBDAddr() )
			{
				/** SppConnect( sppb.spp, &sppb.bd_addr ); doesnt work ? 
					And need to figure out the connect_params setting**/
				
				spp_connect_params config;
				config.size_search_pattern = 0;
				config.search_pattern = 0;
				config.rfcomm_channel_number = 0;
				config.max_frame_size = 0;
			
				SppConnectLazy( &sppb.bd_addr , 1, &sppb.task, &config);
				
				sppb.connecting = TRUE;
				
				/** no need to send time out event here, wait for SPP_CONNECT_CFM.. **/

				ret = TRUE;
				
			}					
		}
	}

	return ret;
	
}


/**************************************************************************************************
  
  initialising state 
  
  */
static void sppb_initialising_state_enter(void) {
	
	Sink sink;
	Source source;
	
	DEBUG(("spp initialising state enter...\n"));

	sink = StreamUartSink();
	source = StreamUartSource();

 
	
	StreamUartConfigure(VM_UART_RATE_115K2, VM_UART_STOP_ONE, VM_UART_PARITY_NONE);
	
	if( source )
	{
	/*	StreamConfigure( VM_STREAM_UART_CONFIG, VM_STREAM_UART_THROUGHPUT);*/
		StreamConfigure( VM_STREAM_UART_CONFIG, VM_STREAM_UART_LATENCY);
		StreamConnectDispose( source );	
		/** initialise connection library **/
		ConnectionInit(getSppbTask());
	}

	/** undisconnect the source/sink **/
	StreamDisconnect(0, sink);
	StreamDisconnect(source, 0);

	/** cancel all more_data messages if any **/
	(void)MessageCancelAll(getSppbTask(), MESSAGE_MORE_DATA);
	
	/** drop all data if any **/
	SourceDrop( source, SourceSize( source ) );
	
    /*	SourceEmpty(source);	*/
		
	
	/** set uart as send more_data/more_space messages only once, see api reference **/
	SourceConfigure( source, VM_SOURCE_MESSAGES, VM_MESSAGES_ALL);
	SinkConfigure( sink, VM_SINK_MESSAGES, VM_MESSAGES_ALL);

	MessageSinkTask(sink, getSppbTask());
}

static void sppb_initialising_state_exit(void) {
	
	DEBUG(("spp initialising state exit...\n"));
    
    MessageCancelAll( &sppb.task , SPPA_CONNECTIONSMFUNC_TIMEOUT);
	/** nothing to do **/
}

static void sppb_initialising_state_handler(Task task, MessageId id, Message message) {

	switch(id) {
		
		case SPP_INIT_CFM:

			DEBUG(("spp initialising state, SPP_INIT_CFM message arrived...\n"));
			
            /* Check for spp_init_success. What do we do if it failed? */
            if (((SPP_INIT_CFM_T *) message)->status == spp_init_success)
            {
                
	            if( haveHostBDAddr() )
	            {
	            	sendRespToUart( UART_RESP_READY_WITHHOSTADD );
                    
                    /** switch to ready state, unconnectable, undiscoverable **/
			    	sppb.spp_initialised = TRUE;				
			    	sppb_initialising_state_exit();
			    	sppb.state = SPPB_READY;
			    	sppb_ready_state_enter();
	            }
	            else
	            {
	            	ConnectionSmGetIndexedAttribute( SPPA_ATTR_PS_BASE, SPPA_DEV_DEFAULT_ADDR_INDEX, 0 );
						
	            	MessageCancelAll( &sppb.task , SPPA_CONNECTIONSMFUNC_TIMEOUT);
	            	MessageSendLater( &sppb.task, SPPA_CONNECTIONSMFUNC_TIMEOUT, 0, SPPA_CONNECTIONSMFUNC_TIMEOUT_DURATION);
	            }                  

            }
			else {
				
				/** don't panic **/
			}
            break;
		case CL_SM_GET_INDEXED_ATTRIBUTE_CFM:
			{
				DEBUG(( "spp ready state, CL_SM_GET_INDEXED_ATTRIBUTE_CFM message arrived...\n" ));

				MessageCancelAll( &sppb.task , SPPA_CONNECTIONSMFUNC_TIMEOUT);
				
				if ( HandleClSmGetIndexedAttributeConfirm( (CL_SM_GET_INDEXED_ATTRIBUTE_CFM_T *)message ) )
				{
					/** get host BD_addr success **/
					sendRespToUart( UART_RESP_READY_WITHHOSTADD ); 
				}
				else		/** get host BD_addr failed, go to pairable state **/
				{
					sendRespToUart( UART_RESP_READY_NOHOSTADD );
				}
                
                /** switch to ready state, unconnectable, undiscoverable **/
			   	sppb.spp_initialised = TRUE;				
			   	sppb_initialising_state_exit();
			    sppb.state = SPPB_READY;
			    sppb_ready_state_enter();
			}
			break;	
			
		case SPPA_CONNECTIONSMFUNC_TIMEOUT:	
			{
				/** get host BD_addr failed, go to pairable state **/
				DEBUG(( "spp ready state, SPPA_CONNECTIONSMFUNC_TIMEOUT message arrived...\n" ));
    			sendRespToUart( UART_RESP_READY_NOHOSTADD );
                /** switch to ready state, unconnectable, undiscoverable **/
                
                sppb.spp_initialised = TRUE;				
			   	sppb_initialising_state_exit();
			    sppb.state = SPPB_READY;
			    sppb_ready_state_enter();
			}
			break;	
		default:
			unhandledSppState(sppb.state, id);
			break;
	}
}

/**************************************************************************************************
  
  ready state
  
  revert state map design, in fact the idle/working external state is the superstate of pairable, connecting, connected and disconnecting.
  
  TODO: if switched back from pairable/connecting state (when powering off), 
	there may be some connect_ind/cfm message in queue, check it and reject them politely
  
  */
static void sppb_ready_state_enter(void) {
         
	DEBUG(("spp ready state enter...\n"));	

 
   
}

static void sppb_ready_state_exit(void) {
	
	DEBUG(("spp ready state exit...\n"));
	
}

static void sppb_ready_state_handler(Task task, MessageId id, Message message) {
		
    uart_cmd_type cmdType;    
    uint8 deviceBuf[K_MAX_DEVICEID_LEN];

	switch(id) {            
		case MESSAGE_MORE_DATA:

			DEBUG(("spp ready_state, MESSAGE_MORE_DATA message arrived...\n"));
			
			cmdType = recieveDataFromUart();

			switch( cmdType )	
			{
                case CMD_INITDEVICEID:
                { 
                    memset( deviceBuf, 0x00, K_MAX_DEVICEID_LEN );
                    sprintf( (char*)deviceBuf, "CTS %010lu", sppb.deviceID);                   
                    ConnectionChangeLocalName( K_DEVICENAME_MAX_LEN, deviceBuf ); 
                    
                    sendRespToUart( UART_RESP_READY_INITDEVICEID );
                }
                break;
                    
			    case CMD_PAIR:
			    {
			        sppb_ready_state_exit();
				    sendRespToUart( UART_RESP_PAIRCMD_RECIEVED );
				    setSppState(SPPB_PAIRABLE);
				    sppb_pairable_state_enter();
				}
			    break;
					
				case CMD_GOCONNECTABLE:
				{
					sppb_ready_state_exit();
					sendRespToUart(UART_RESP_CONNECTABLE_SUCCESS);
					setSppState(SPPB_DISCONNECTED);
					sppb_disconnected_state_enter();				
				}
				break;
				case CMD_QUITBT:
				{
				/*	sppb_ready_state_exit();*/
				}
				break;
				default:
					break;
			}
			break;


		default:
			unhandledSppState(sppb.state, id);
			break;			
	}
}



/**************************************************************************************************
  
  connectAsSPPA state
  
  */

static void sppb_connectAsSPPA_state_enter(void) {
	
	DEBUG(("spp connectAsSPPA state enter...\n"));	


	ConnectionSmRegisterIncomingService(0x0000, 	
										0x0001, 
										0x0000);
	/* Write class of device */
	ConnectionWriteClassOfDevice(CLASS_OF_DEVICE);
	ConnectionSmSetSdpSecurityIn(TRUE);
	
	sppb.connecting = FALSE;

	if ( haveHostBDAddr() )		/** avavilable bd_addr, connect the host **/
	{
		if ( !ConnectHostAsSppA() )		
		{
			/** connection function failed, go to disconnected state **/
			sppb_connectAsSPPA_state_exit();
			sendRespToUart( UART_RESP_FAILED );
			setSppState(SPPB_DISCONNECTED);
			sppb_disconnected_state_enter();
		}
	}
	else
	{
		/** connection function failed, go to disconnected state **/
		sppb_connectAsSPPA_state_exit();
		sendRespToUart( UART_RESP_FAILED );
		setSppState(SPPB_DISCONNECTED);
		sppb_disconnected_state_enter();
	}
	
 	
}

static void sppb_connectAsSPPA_state_exit(void) {
	
	DEBUG(("spp connectAsSPPA state exit...\n"));
	
	sppb.connecting = FALSE;

}

static void sppb_connectAsSPPA_state_handler(Task task, MessageId id, Message message) {
	
	switch(id) 
	{
		case MESSAGE_MORE_DATA:

			DEBUG(("spp sppb_connectAsSPPA_state_handler, MESSAGE_MORE_DATA message arrived...\n"));
			
			{
				uart_cmd_type cmdType = recieveDataFromUart();
				
				switch( cmdType )
				{
					case CMD_QUITBT:
					{
                        SppDisconnect(sppb.spp);
						sppb_connectAsSPPA_state_exit();
						setSppState(SPPB_READY);
						sppb_ready_state_enter();
		
					}
					break;
					default:
						break;
				}
			}
			break;
		case SPP_CONNECT_CFM:
			{		
				SPP_CONNECT_CFM_T *cfm = (SPP_CONNECT_CFM_T *) message;
				DEBUG(("spp connectAsSPPA state, SPP_CONNECT_CFM message arrived... \n"));

				if (cfm ->status == rfcomm_connect_success) {
												
					sppb.spp = cfm->spp;
					sppb.spp_sink = cfm ->sink;
						
					sppb_connectAsSPPA_state_exit();
					sendRespToUart( UART_RESP_CONNECTBYSPPA_SUCCESS );
					setSppState(SPPB_CONNECTED);
					sppb_connected_state_enter();
				}
				else {
					sppb_connectAsSPPA_state_exit();
					sendRespToUart( UART_RESP_FAILED );
					setSppState(SPPB_DISCONNECTED);
					sppb_disconnected_state_enter();
					break;
				}	
				
			}
			break;
			
		case CL_DM_ACL_OPENED_IND:
			/* in case, nokia E6 will reture CL_DM_ACL_OPENED_IND message coz' L2CAP open 
			sppb_connectAsSPPA_state_exit();
			setSppState(SPPB_DISCONNECTED);
			sppb_disconnected_state_enter();*/
			break;		

	}
}



/**************************************************************************************************
  
  pairable state
  
  */

static void sppb_pairable_state_enter() {

	DEBUG(("spp pairable state enter...\n"));
	
	/** the code is from original bluelab sample in spp_dev_b spp_dev_inquire() **/
	/* don't know if some initialisation could be done multiple times */
	/* Turn off security */
	ConnectionSmRegisterIncomingService(0x0000, 	
										0x0001, 
										0x0000);
	/* Write class of device */
	ConnectionWriteClassOfDevice(CLASS_OF_DEVICE);
	/* Start Inquiry mode */
	/** setSppState(SPPB_PAIRABLE); **/
	/* Set devB device to inquiry scan mode, waiting for discovery */
	ConnectionWriteInquiryscanActivity(0x400, 0x200);
	ConnectionSmSetSdpSecurityIn(TRUE);

	/* Delete all trusted devices */
 	ConnectionSmDeleteAllAuthDevices(SPPA_ATTR_PS_BASE);
	BdaddrSetZero( &sppb.bd_addr ); 

	/** Make this device discoverable (inquiry scan), and connectable (page scan) **/
	ConnectionWriteScanEnable( hci_scan_enable_inq_and_page );	

	MessageSendLater(getSppbTask(), SPPB_PAIRABLE_TIMEOUT_IND, 0, SPPB_PAIRABLE_DURATION);

 
}


static void sppb_pairable_state_exit() {

	DEBUG(("spp pairable state exit...\n"));

	/* clean temp bd addr **/
	BdaddrIsZero( &sppb.tempBd_addr );

	/* turn off scan **/
	ConnectionWriteScanEnable(hci_scan_enable_off);
	

	MessageCancelAll(getSppbTask(), SPPB_PAIRABLE_TIMEOUT_IND);	
	
}

		
static void sppb_pairable_state_handler(Task taks, MessageId id, Message message) {
	
	switch(id) {
		
		case CL_SM_AUTHENTICATE_CFM:
			{
				CL_SM_AUTHENTICATE_CFM_T* cfm = ( CL_SM_AUTHENTICATE_CFM_T* )message;
			
			 	DEBUG(("spp pairable state, CL_SM_AUTHENTICATE_CFM message arrived...\n"));

				if( cfm->status == auth_status_success)	/** pair success  **/
				{
					UpdateNewPairedDevice( &cfm->bd_addr );
					sppb_pairable_state_exit();
					sppb.state = SPPB_DISCONNECTED;
					sendRespToUart( UART_RESP_PAIR_SUCCESS );
					sppb_disconnected_state_enter();
				}
				else
				{
					DEBUG(("Pairing failed\n"));
					BdaddrIsZero( &sppb.tempBd_addr );
				}
			}
			break;
		case SPPB_PAIRABLE_TIMEOUT_IND:
			
			DEBUG(("spp pairable state, SPPB_PAIRABLE_TIMEOUT_IND message arrived...\n"));
			
			sppb_pairable_state_exit();
			sendRespToUart( UART_RESP_PAIR_TIMEOUT );
			sppb.state = SPPB_IDLE;
			sppb_idle_state_enter();

			break;

		case MESSAGE_MORE_DATA:

			DEBUG(("spp pairable, MESSAGE_MORE_DATA message arrived...\n"));
			
			{
				uart_cmd_type cmdType = recieveDataFromUart();
				
				switch( cmdType )
				{
					case CMD_QUITBT:
					{
						sppb_pairable_state_exit();
						setSppState(SPPB_READY);
						sppb_ready_state_enter();
		
					}
					break;
					default:
						break;
				}
			}
			break;

	}
}


/**************************************************************************************************
  
  Idle state
  
  */
static void sppb_idle_state_enter(void) {
	
	DEBUG(("spp idle state enter...\n"));	

 	
}

static void sppb_idle_state_handler(Task task, MessageId id, Message message) {
	
	unhandledSppState(sppb.state, id);
	
}



/**************************************************************************************************
  
  disconnected super-state
  
  */

static void sppb_disconnected_state_enter() {

	DEBUG(("spp disconnected state enter...\n"));
	
	sppb.connecting = FALSE;
	
    ConnectionWriteScanEnable(hci_scan_enable_page);	
	
 
}

static void sppb_disconnected_state_exit() {

	DEBUG(("spp disconnected state exit...\n"));

	/* turn off scan **/
	ConnectionWriteScanEnable(hci_scan_enable_off);
	
}

static void sppb_disconnected_state_handler(Task taks, MessageId id, Message message) {
	
	switch(id) {
		
		case SPP_CONNECT_IND:
		
			DEBUG(("spp disconnected state, SPP_CONNECT_IND message arrived... \n"));
			
			/** only accept connection in disconnected sub-state **/
			if (sppb.connecting == FALSE) {
				
				sppDevAuthoriseConnectInd(&sppb,(SPP_CONNECT_IND_T*)message);
				sppb.connecting = TRUE;
			 
			}
			else {
				
				/** should we respond with rejection ??? **/
			}
			
			break;
			
		case SPP_CONNECT_CFM:
			
			DEBUG(("spp disconnected state, SPP_CONNECT_CFM message arrived... \n"));
			
			if (sppb.connecting == FALSE) 
			{	/** disconnected state **/

				SPP_CONNECT_CFM_T *cfm = (SPP_CONNECT_CFM_T *) message;
				if (cfm->status == rfcomm_connect_success)
				{
					/* Device has been reset to pairable mode. Disconnect from current device, this is code from official sppb example */
					SppDisconnect(cfm->spp);
				}					
			}
			else 
			{	/** connecting state **/

				SPP_CONNECT_CFM_T *cfm = (SPP_CONNECT_CFM_T *) message;

				if (cfm ->status == rfcomm_connect_success) 
				{

					sppb.spp = cfm->spp;
					sppb.spp_sink = cfm ->sink;

					MessageCancelAll( &sppb.task , SPPB_CONNECT_RESP_DELAY);
					MessageSendLater( &sppb.task, SPPB_CONNECT_RESP_DELAY, 0, SPPB_CONNECT_RESP_DELAY_DURATION);

				}
				else
				{
					sppb.connecting = FALSE;
				 
				}	
			}
			
			break;
		case SPPB_CONNECT_RESP_DELAY:
			{
				MessageCancelAll( &sppb.task , SPPB_CONNECT_RESP_DELAY);
				
				sppb_disconnected_state_exit();
				setSppState(SPPB_CONNECTED);
				sendRespToUart( UART_RESP_CONNECTED_SUCCESS );
				sppb_connected_state_enter();
			}
			break;
			
	
		case MESSAGE_MORE_DATA:

			DEBUG(("spp ready_state, MESSAGE_MORE_DATA message arrived...\n"));
			
			{
				uart_cmd_type cmdType = recieveDataFromUart();
				
				switch( cmdType )
				{
					case CMD_PAIR:
					{
                        sppb_disconnected_state_exit();
						sendRespToUart( UART_RESP_PAIRCMD_RECIEVED );
						setSppState(SPPB_PAIRABLE);
						sppb_pairable_state_enter();
					}
					break;
					case CMD_QUITBT:
					{
						sppb_disconnected_state_exit();
						setSppState(SPPB_READY);
						sppb_ready_state_enter();     
    				}
					break;
					case CMD_CONNECT:
					{
						if( haveHostBDAddr() )
						{
							sppb_disconnected_state_exit();
							setSppState(SPPB_CONNECTAS_SPPA);
							sppb_connectAsSPPA_state_enter();
						}
					}
					break;
					default:
						break;
				}
			}
			break;

	}
}





/**************************************************************************************************
  
  connected state
  
  */
static void sppb_connected_state_enter() {

	Sink sink;
	Source source;
	
	DEBUG(("spp connected state enter...\n"));
	
 
	
	sink = sppb.spp_sink;
	source = StreamUartSource();
	
	/* StreamConnectDispose(StreamSourceFromSink(sink)); */

		/** set uart as send more_data/more_space messages only once, see api reference **/
	SourceConfigure( source, VM_SOURCE_MESSAGES, VM_MESSAGES_SOME);
	SinkConfigure( sink, VM_SINK_MESSAGES, VM_MESSAGES_SOME);  
    
	sppb.dataRecieving = FALSE;
	
}


static void sppb_connected_state_exit() {

	DEBUG(("spp connected state exit...\n"));

	sppb.spp_sink = 0;	
	/** dont clear sppb.spp, the next state need it, it covers both connected state AND disconnecting state **/
}

static void sppb_connected_state_handler(Task task, MessageId id, Message message) {
	
	switch(id) {
#if 1
		case SPP_MESSAGE_MORE_DATA:
		{
			Source source;
			uint16 size;
			const uint8* buf;
						
			DEBUG(("spp connected state, SPP_MESSAGE_MORE_DATA message arrived...\n"));
			
			source = StreamSourceFromSink(sppb.spp_sink);
			size = SourceSize(source);	/** size won't be zero **/
			buf = SourceMap(source);			

			send( StreamUartSink(), buf , size );
            
            SourceDrop(source, size);
		}
		break;
#endif
        
		case SPP_DISCONNECT_IND:	/** passively disconnected, switching to scan **/

			DEBUG(("spp connected state, SPP_DISCONNECT_IND message arrived...\n"));
			
			sppb_connected_state_exit();
			setSppState(SPPB_DISCONNECTED);

			sendRespToUart(UART_RESP_HOST_LOST);
			
			sppb_disconnected_state_enter();
			break;
			
		case HAL_MESSAGE_SWITCHING_OFF:
			
			DEBUG(("spp connected state, HAL_MESSAGE_SWITCHING_OFF message arrived...\n"));
			
			sppb_connected_state_exit();
			setSppState(SPPB_DISCONNECTING);
			sppb_disconnecting_state_enter();
			
			break;
		case MESSAGE_MORE_DATA:

			DEBUG(("spp ready_state, MESSAGE_MORE_DATA message arrived...\n"));
			{
				Source source;
				uint16 size;
				const uint8* ptr;
				uart_cmd_type cmdType;

				cmdType = CMD_UNKOWN;
				source = StreamUartSource();
				size = SourceSize(source);

				DEBUG(( "    source has %d bytes of data... \n", size ));
				ptr = SourceMap(source);

				if( FALSE == sppb.dataRecieving ) 	/* command mode  */
				{
					if( size >= K_Cmd_Len )
					{
						cmdType = processUartData( ptr, K_Cmd_Len);

						switch( cmdType )
						{
							case CMD_SEND:
							{
                                uint16 dataLen;
                                uint16 posCMDHeadLen = K_Cmd_Len + K_Cmd_SendDataLength_Len;
								dataLen = size - posCMDHeadLen;

								sppb.dataRecieving = TRUE;
								getDataBodyLen(ptr);
                                
								if ( dataLen && SinkIsValid(sppb.spp_sink) && (SinkSlack(sppb.spp_sink) >= dataLen) ) 
								{
                                    sppb.dataRecieving = FALSE;
									sendDataToSpp( ptr + posCMDHeadLen, dataLen );
									sendRespToUart(UART_RESP_SEND_SUCCESS);	
								}
								else
								{
                                    sppb.dataRecieving = FALSE;
									DEBUG( ("send failed, spp is not ready \n") );
									sendRespToUart(UART_RESP_SEND_FAILED);
								}
								/*MessageCancelAll( &sppb.task , SPPB_PIPE_SENDDATA_TIMEOUT);
								MessageSendLater( &sppb.task, SPPB_PIPE_SENDDATA_TIMEOUT, 0, SPP_SENDDATA_TIMEOUT_DURATION);
							    */
                            }
							break;
							
							case CMD_DISCONNECT:
							{
								sppb_connected_state_exit();
								setSppState(SPPB_DISCONNECTING);
								sppb_disconnecting_state_enter();
						
							}
							break;

							case CMD_QUITBT:
							{
								sppb_connected_state_exit();
								setSppState(SPPB_DISCONNECTING);
								sppb_disconnecting_state_enter();
						
							}
							break;

							default:
								break;
						}

					}					
				}
				else 							
				{
					sendDataToSpp( ptr, size );
				}
			
				SourceDrop(source, size);
			}
			break;
			
			case SPPB_PIPE_SENDDATA_TIMEOUT:
				{
					
					sppb.dataRecieving = FALSE;
					sendRespToUart(UART_RESP_SEND_SUCCESS);
				}
				break;

		default:
			
			unhandledSppState(sppb.state, id);
			break;
	}					 
}

static void sppb_disconnecting_state_enter(void) {
	
	DEBUG(("spp disconnecting state enter...\n"));
	
 
	/** check reason and output debug **/
	SppDisconnect(sppb.spp);
}

static void sppb_disconnecting_state_exit(void) {
	
	DEBUG(("spp disconnecting state exit...\n"));
	/** nothing to do **/
}

static void sppb_disconnecting_state_handler(Task task, MessageId id, Message message) {
	
	switch(id) {
		
		case SPP_DISCONNECT_IND:
		
			DEBUG(("spp disconnecting state, SPP_DISCONNECT_IND message arrived...\n"));
			sppb_disconnecting_state_exit();
			setSppState(SPPB_DISCONNECTED);

			sendRespToUart(UART_RESP_DISCONNECT_SUCCESS);
			sppb_disconnected_state_enter();
			
			break;
		
		default:
			unhandledSppState(sppb.state, id);
			break;		
	}
}

static void sppb_handler(Task task, MessageId id, Message message) {
	
	sppb_state_t state = sppb.state;
	
	if ((id & 0xFF00) == CL_MESSAGE_BASE) {
		
		cl_handler(task, id, message);
		
		/** deleted by mark, profile handler need to handle some messages about connectionSM **/
		/**	return;**/
	}
	
	switch (state) {
		
		case SPPB_INITIALISING:
			sppb_initialising_state_handler(task, id, message);
			break;
		case SPPB_READY:
			sppb_ready_state_handler(task, id, message);
			break;
		case SPPB_CONNECTAS_SPPA:
			sppb_connectAsSPPA_state_handler( task, id, message );
			break;
		case SPPB_DISCONNECTED:
			sppb_disconnected_state_handler( task, id, message );
			break;
		case SPPB_IDLE:
			sppb_idle_state_handler( task, id, message );
			break;			
		case SPPB_PAIRABLE:
			sppb_pairable_state_handler( task, id, message );
			break;							
		case SPPB_CONNECTED:
			sppb_connected_state_handler(task, id, message);
			break;
			
		case SPPB_DISCONNECTING:
			sppb_disconnecting_state_handler(task, id, message);
			break;
			
		default:
			break;
	}
}


/** connection library layer **/
static void cl_handler(Task task, MessageId id, Message message) {
	
	switch (id) {	
	case CL_INIT_CFM:
        DEBUG(("CL_INIT_CFM\n"));
        if(((CL_INIT_CFM_T*)message)->status == success) {
			
			/** change sub-state **/
			sppb.cl_initialised = TRUE;
            sppDevInit();   
		}
        else {
			
            /** don't panic **/
		}
        break;
    case CL_DM_LINK_SUPERVISION_TIMEOUT_IND:
        DEBUG(("CL_DM_LINK_SUPERVISION_TIMEOUT_IND\n"));
        break;
    case CL_DM_SNIFF_SUB_RATING_IND:
        DEBUG(("CL_DM_SNIFF_SUB_RATING_IND\n"));
        break;
	
    case CL_DM_ACL_OPENED_IND:
        DEBUG(("CL_DM_ACL_OPENED_IND\n"));
        break;
    case CL_DM_ACL_CLOSED_IND:
        DEBUG(("CL_DM_ACL_CLOSED_IND\n"));
        break;
    case CL_SM_PIN_CODE_IND:
        DEBUG(("CL_SM_PIN_CODE_IND\n"));
        sppDevHandlePinCodeRequest((CL_SM_PIN_CODE_IND_T *) message);
        break;
    case CL_SM_AUTHORISE_IND:  
        DEBUG(("CL_SM_PIN_CODE_IND\n"));
        sppDevAuthoriseResponse((CL_SM_AUTHORISE_IND_T*) message);
        break;
    case CL_SM_ENCRYPTION_KEY_REFRESH_IND:
        DEBUG(("CL_SM_ENCRYPTION_KEY_REFRESH_IND\n"));
        break;
    case CL_DM_LINK_POLICY_IND:
        DEBUG(("CL_DM_LINK_POLICY_IND\n"));
        break;
    case CL_SM_IO_CAPABILITY_REQ_IND:
        DEBUG(("CL_SM_IO_CAPABILITY_REQ_IND\n"));
        ConnectionSmIoCapabilityResponse( &sppb.tempBd_addr, 
                                          cl_sm_io_cap_no_input_no_output,
                                          FALSE,
                                          TRUE,
                                          FALSE,
                                          0,
                                          0 );
        break;
 
    case CL_SM_REMOTE_IO_CAPABILITY_IND:
        {
            CL_SM_REMOTE_IO_CAPABILITY_IND_T *csricit = 
                    ( CL_SM_REMOTE_IO_CAPABILITY_IND_T *) message;

            DEBUG(("CL_SM_REMOTE_IO_CAPABILITY_REQ_IND\n"));
            DEBUG(("\t Remote Addr: nap %04x uap %02x lap %08lx\n",
                    csricit->bd_addr.nap,
                    csricit->bd_addr.uap,
                    csricit->bd_addr.lap ));
            sppb.tempBd_addr = csricit->bd_addr;
        }
        break;
		
	default:
		break;
	}
}

void sppb_init(void) {
	
	
	sppb.task.handler = sppb_handler;
/**	sppb.hal_task = hal_task;**/
	
	sppb.spp = 0;
	sppb.spp_sink = 0;

	BdaddrIsZero( &sppb.bd_addr );
	BdaddrIsZero( &sppb.tempBd_addr );
	
	sppb.cl_initialised = FALSE;
	sppb.spp_initialised = FALSE;
	sppb.dataRecieving = FALSE;
	sppb.dataRecieveCounter = 0;
	
	sppb.state = SPPB_INITIALISING;
	sppb_initialising_state_enter();
}














