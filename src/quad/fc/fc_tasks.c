
#include "fc_tasks.h"
#include "fc_core.h"
#include "fc_rc.h"

void taskUpdateRxMain(timeUs_t currentTimeUs)			// TODO: make this function static for rtos tasks assignment
//static void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	/* retrieve RX data */
	processRx(currentTimeUs);
	
	/* new rx data is available */
	isRXDataNew = true;
	
	/* updateRcCommands function sets rcCommand  */
	updateRcCommands();
	
	/* update LEDs */
//	updateLEDs();
}
