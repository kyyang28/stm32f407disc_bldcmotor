
#include "configMaster.h"

void readEEPROM(void)
{
	/* Sanity check */
	
	
//	suspendRxSignal();
	
	/* Read flash */
	
	/* set profile to the first one */
	setProfile(masterConfig.current_profile_index);
	
	validateAndFixConfig();
	activateConfig();
	
//	resumeRxSignal();
}
