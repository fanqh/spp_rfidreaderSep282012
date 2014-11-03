#include "hal.h"
#include "hal_private.h"
#include "sppb.h"
/*#include "scanner.h"*/
#include "spp_dev_private.h"
#include "indication.h"
#include "spp_dev_b_leds.h"
/*
static uint16 currentState = IND_NONE;
*/

#define	K_VOLTAG_HIGHVALUE		3900
#define	K_VOLTAG_MODESTVALUE		3600

uint16 calcIndication(void);
bool IsVoltageFine(void);
bool IsVoltagLow(void);

bool IsVoltageFine(void)
{
	halTaskData* hal_task;
	
	bool ret = FALSE;
	
	hal_task = (halTaskData*)getHalTask();

	if ( hal_task )
	{
		ret = hal_task ->voltage > K_VOLTAG_HIGHVALUE;
	}
	
	return ret;
}
bool IsVoltagLow(void)
{
	halTaskData* hal_task;
	
	bool ret = FALSE;
	
	hal_task = (halTaskData*)getHalTask();

	if ( hal_task )
	{
		ret = hal_task ->voltage < K_VOLTAG_MODESTVALUE;
	}
	
	return ret;
}


uint16 calcIndication(void) {

	halTaskData* hal_task;
	sppb_task_t* sppb_task;
			
	hal_task = (halTaskData*)getHalTask();
	sppb_task = (sppb_task_t*)getSppbTask();

	switch(hal_task ->state)
	{
		case INITIALISING:

			if (hal_task ->voltage == 0xFFFF || hal_task ->charging_state == CHARGING_UNKNOWN) {

				return IND_UNKNOWN;
			}
			else {

				if ( IsVoltageFine()) {

					return IND_BATT_FINE;
				}
				else if ( IsVoltagLow() ) {

					return IND_BATT_LOW;
				}
				else {
					return IND_BATT_MODEST;
				}
			}
			break;
			
		case ACTIVATING:
			
			if (hal_task ->beep_finished == FALSE) {

				return IND_NONE;
			}
			else {
				
				return IND_ACTIVATING_WAITING;
			}
			break;
		
		case ACTIVE:
			
			switch (sppb_task ->state) {
				
				case SPPB_INITIALISING:
				case SPPB_READY:
					
					return IND_NONE;
					break;
					
				case SPPB_PAIRABLE:
					return IND_DISCOVERABLE;
					break;

				case SPPB_DISCONNECTED:
					
					if (hal_task ->charging_state == CHARGING_CHARGING) {

						if ( IsVoltageFine() ) {
							
							return IND_CABLEDDISCONNECTED_CHARGE_FINE;
						}
						else if ( IsVoltagLow()  ) {

							return IND_CABLEDDISCONNECTED_CHARGE_LOW;
						}
						else {
							return IND_CABLEDDISCONNECTED_CHARGE_MODEST;
						}
	
					}
					else
					{
					
						if ( IsVoltagLow() ) {
							
							return IND_CABLEDDISCONNECTED_BATT_LOW;
						}
						else {
							return IND_CABLEDDISCONNECTED;
						}
					}
					break;
				case SPPB_IDLE:
					if ( IsVoltageFine() ) {
						
						return IND_IDLE_CHARGE_FINE;
					}
					else if ( IsVoltagLow() ) {

						return IND_IDLE_BATT_LOW;
					}
					else {
						return IND_IDLE_CHARGE_MODEST;
					}
					break;
					
				case SPPB_CONNECTED:
				case SPPB_DISCONNECTING:

					if (hal_task ->charging_state == CHARGING_CHARGING) {
						
						if ( IsVoltageFine() ) {
							
							return IND_CABLEDCONNECTED_CHARGE_FINE;
						}
						else if (  IsVoltagLow() ) {

							return IND_CABLEDCONNECTED_CHARGE_LOW;
						}
						else {
							return IND_CABLEDCONNECTED_CHARGE_MODEST;
						}
					}
					else {
						
						if ( IsVoltagLow() ) {
							
							return IND_CABLEDCONNECTED_BATT_LOW;
						}
						else {
							return IND_CABLEDCONNECTED;
						}
					}
					break;
					
				case SPPB_CONNECTAS_SPPA:
					return IND_CABLEDCONNECTING;
					break;
			}
			
			break;
			
		case DEACTIVATING:

			return IND_NONE;
			break;
	}

	/** should not go here **/
	return IND_NONE;
}

void update_indication(void) {
	/*
	uint16 state = calcIndication();

	if (state == currentState) {

		return;
	}
	
	currentState = state;
	
	ledsPlay(currentState);
	*/
}

#if 0

	/** code from hid project **/

uint16 calcIndication(void) {
	
	halTaskData* 	hal_task;
	appTaskData* 	hid_task;
	scanner_task_t* scanner_task;
	
	hal_task = (halTaskData*)getHalTask();
	hid_task = (appTaskData*)getHidKBTask();
/*	scanner_task = (scanner_task_t*)getScannerTask();*/

	switch(hal_task ->state)
	{
		case INITIALISING:

			if (hal_task ->voltage == 0xFFFF || hal_task ->charging_state == CHARGING_UNKNOWN) {

				return IND_UNKNOWN;
			}
			else {

				if (hal_task ->voltage > 3900) {

					return IND_BATT_FINE;
				}
				else if (hal_task ->voltage > 3600) {

					return IND_BATT_MODEST;
				}
				else {
					return IND_BATT_LOW;
				}
			}
			break;
			
/*		case ACTIVATING:

			return IND_NONE;
			break;
	*/		
		case ACTIVE:
			
			if ( scanner_task ->state == SCANNER_ON && scanner_task ->on_state == SCANNER_ON_SCANNING ) {
				
				return IND_SCANNING;	/** this thing override anything else **/
			}

			switch( hid_task ->state ) 
			{
				case appInitialising:
				
					return IND_NONE;
					break;

				case appDiscoverable:
				case appDiscoverableConnecting:
				case appDiscoverableConnected:
					
					return IND_DISCOVERABLE;
					break;
					
				case appCabledConnecting:
					return IND_CABLEDCONNECTING;
					break;
					
				case appCabledDisconnecting:
				case appCabledDisconnected:
					
					if (hal_task ->charging_state == CHARGING_CHARGING) {
						
						if (hal_task ->voltage > 3900) {
							
							return IND_CABLEDDISCONNECTED_CHARGE_FINE;
						}
						else if (hal_task ->voltage > 3600) {

							return IND_CABLEDDISCONNECTED_CHARGE_MODEST;
						}
						else {
							return IND_CABLEDDISCONNECTED_CHARGE_LOW;
						}
					}
					else {
						
						if (hal_task ->voltage > 3600) {
							
							return IND_CABLEDDISCONNECTED;
						}
						else {
							return IND_CABLEDDISCONNECTED_BATT_LOW;
						}
					}
					break;
					
				case appCabledConnected:
					
					if (hal_task ->charging_state == CHARGING_CHARGING) {
						
						if (hal_task ->voltage > 3900) {
							
							return IND_CABLEDCONNECTED_CHARGE_FINE;
						}
						else if (hal_task ->voltage > 3600) {

							return IND_CABLEDCONNECTED_CHARGE_MODEST;
						}
						else {
							return IND_CABLEDCONNECTED_CHARGE_LOW;
						}
					}
					else {
						
						if (hal_task ->voltage > 3600) {
							
							return IND_CABLEDCONNECTED;
						}
						else {
							return IND_CABLEDCONNECTED_BATT_LOW;
						}
					}
					break;
					
				case appIdle:
					
					if (hal_task ->charging_state == CHARGING_CHARGING) {
						
						if (hal_task ->voltage > 3900) {
							
							return IND_IDLE_CHARGE_FINE;
						}
						else if (hal_task ->voltage > 3600) {

							return IND_IDLE_CHARGE_MODEST;
						}
						else {
							return IND_IDLE_CHARGE_LOW;
						}
					}
					else {
						
						if (hal_task ->voltage > 3600) {
							
							return IND_IDLE;
						}
						else {
							return IND_IDLE_BATT_LOW;
						}
					}
					break;
					
				default:
					
					break;
			}
			
			break;
			
		case DEACTIVATING:

			return IND_NONE;
			break;
	}

	/** should not go here **/
	return IND_NONE;
}

#endif



















