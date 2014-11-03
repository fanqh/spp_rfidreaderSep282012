#include "debug.h"
#include "hal.h"
#include "sppb.h"


int main(void)
{					
	hal_init();
	sppb_init();
	
	MessageLoop();	
    
    return 0;
}



