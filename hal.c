#include <csrtypes.h>
#include <battery.h>
#include <pio.h>
#include <panic.h>
#include "spp_dev_b_buttons.h"
#include "spp_dev_b_leds.h"
#include "hal.h"
#include "hal_config.h"
#include "hal_private.h"
#include "errman.h"
#include "debug.h"

#define BEEP_ON_TIME	300
#define BEEP_OFF_TIME	100

#define BEEP_ONCE_DURATION 		(BEEP_ON_TIME + BEEP_OFF_TIME)
#define BEEP_TWICE_DURATION		((BEEP_ON_TIME + BEEP_OFF_TIME) * 2)
#define BEEP_THREE_TIME_DURATION	((BEEP_ON_TIME + BEEP_OFF_TIME) * 3)

#define BT_ALIVE_MASK				(1 << 11)

#define K_SWITCH_TESTBOARD

/** task **/
halTaskData hal;


/** hal task handler **/
static void hal_handler(Task task, MessageId id, Message message);

static void hal_set_state(hal_state_t state);

/** state event handlers **/
static void hal_initialising_handler(Task task, MessageId id, Message message);
static void hal_activating_handler(Task task, MessageId id, Message message);
static void hal_active_handler(Task task, MessageId id, Message message);
static void hal_deactivating_handler(Task task, MessageId id, Message message);

/** state entry/exit functions **/
static void hal_initialising_state_enter(void);
static void hal_initialising_state_exit(void);

static void hal_activating_state_enter(void);
static void hal_activating_state_exit(void);

static void hal_active_state_enter(void);
static void hal_active_state_exit(void);

static void hal_deactivating_state_enter(void);
static void hal_deactivating_state_exit(void);

/** fully initialized means charging state, voltage, and bluetooth status all initialized **/
/** bool isFullyInitialized(void); **/

#ifndef K_SWITCH_TESTBOARD
static void enableBTAlive(void);
#endif


static void pio_raw_handler(Message message);
	
/** get hal task **/
Task getHalTask() {
	
	return &hal.task;
}
#ifndef K_SWITCH_TESTBOARD
static void enableBTAlive(void)
{
	/** drive pio high **/
	PioSetDir(BT_ALIVE_MASK, BT_ALIVE_MASK);
	PioSet(BT_ALIVE_MASK, BT_ALIVE_MASK);
}
#endif
static void hal_set_state(hal_state_t state) {
	
	switch(hal.state) {
		
		case INITIALISING:
			
			hal_initialising_state_exit();
			break;	
		
		case ACTIVATING:
			
			hal_activating_state_exit();
			break;
		
		case ACTIVE:
			
			hal_active_state_exit();
			break;
		
		case DEACTIVATING:
			
			hal_deactivating_state_exit();
			break;
		
		default:
			break;
	}
	
	hal.state = state;
	
	switch(state) {
		
		case INITIALISING:
		
			hal_initialising_state_enter();
			break;
		
		case ACTIVATING:
			
			hal_activating_state_enter();
			break;
		
		case ACTIVE:
			
			hal_active_state_enter();
			break;
		
		case DEACTIVATING:
			
			hal_deactivating_state_enter();
			break;
		
		default:
			break;
	}
}

/** hal task handler **/
static void hal_handler(Task task, MessageId id, Message message) {

	/** old pattern **/
	switch (hal.state) {
		
		case INITIALISING:
			hal_initialising_handler(task, id, message);
			break;
			
		case ACTIVATING:
			hal_activating_handler(task, id, message);
			break;
			
		case ACTIVE:
			hal_active_handler(task, id, message);
			break;
			
		case DEACTIVATING:
			hal_deactivating_handler(task, id, message);
			break;
	}
}

static void hal_initialising_state_enter(void) {
	
	DEBUG(("hal initialising state enter...\n"));
	
	/** init pio **/
	pioInit(&hal.pio_state, getHalTask());
	  
}

static void hal_initialising_state_exit(void) {

	DEBUG(("hal initialising state exit...\n"));
}

static void hal_initialising_handler(Task task, MessageId id, Message message) {
	
	switch (id) {
		
		case PIO_RAW:
			{
				DEBUG(("hal initialising state, PIO_RAW message arrived...\n"));
			
				/** update charging state, and no check, even battery low we have nothing to do **/
				pio_raw_handler(message);
				  
			}
			break;					
		default:
			break;
	}
}

static void hal_activating_state_enter(void) {
	
	DEBUG(("hal activating state enter...\n"));
	  	
}

static void hal_activating_state_exit(void) {
	
	DEBUG(("hal activating state exit...\n"));
	

}

static void hal_activating_handler(Task task, MessageId id, Message message) {
	
	switch(id) {
		
		case PIO_RAW:
		
			DEBUG(("hal activating state, PIO_RAW message arrived...\n"));
			
			pio_raw_handler(message);
			
			break;
		case HAL_ACTIVATING_TIMEOUT:
			
			DEBUG(("hal activating state, HAL_ACTIVATING_TIMEOUT message arrived...\n"));
			break;
	}
}

static void hal_active_state_enter(void) {
	
	DEBUG(("hal active state enter...\n"));
	  
}

static void hal_active_state_exit(void) {
	
	DEBUG(("hal active state exit...\n"));
}

static void hal_active_handler(Task task, MessageId id, Message message) {
	
	switch (id) {
		
		case PIO_RAW:
		
			DEBUG(("hal active state, PIO_RAW message arrived...\n"));
			
			pio_raw_handler(message);
			
			break;		
		case HAL_ACTIVE_AUTO_SHUTDOWN_TIMEOUT:
			
			DEBUG(("hal active state, HAL_ACTIVE_AUTO_SHUTDOWN_TIMEOUT message arrived ... \n"));
			
			hal_set_state(DEACTIVATING);
			break;
	}
}

static void hal_deactivating_state_enter(void) {
	
	DEBUG(("hal deactivating state enter...\n"));
	
	  
	/** send message to profile **/
	MessageSend(hal.profile_task, HAL_MESSAGE_SWITCHING_OFF, 0);
}

static void hal_deactivating_state_exit(void) {
	
	DEBUG(("hal deactivating state exit...\n"));
}

static void hal_deactivating_handler(Task task, MessageId id, Message message) {
	
	/** no need to react to any message except timeout, after all, we are going to panic **/
	
	switch(id) {
			
		case HAL_DEACTIVATING_TIMEOUT:
			
			DEBUG(("hal deactivating state, HAL_DEACTIVATING_TIMEOUT message arrived...\n"));
			
			/** brute way !!! */
			Panic();
			break;
	}
}


void hal_init(void) {
	
	/** set task hander Task profileTask, Task functionTask**/
	hal.task.handler = hal_handler;	
	/** set init state **/
	hal.state = INITIALISING;
#ifndef K_SWITCH_TESTBOARD	
	enableBTAlive();
#endif	
	hal_initialising_state_enter();
}

/************
  
  common functions 
  
  ************/

/** this function may need further refine **/
static void pio_raw_handler(Message message) {

	PIO_RAW_T* pio_raw = (PIO_RAW_T*)message;
	
	hal.charging_state = (pio_raw ->pio & PIO_CHARGE_DETECTION) ? CHARGING_CHARGING : CHARGING_NOT_CHARGING;
}















