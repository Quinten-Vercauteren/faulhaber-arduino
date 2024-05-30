#ifndef MCTIMER_H
#define MCTIMER_H

/*------------------------------------------------
 * a class which combines the EveryTimerB as the
 * low level timer and the ISR_Timer to a 
 * versatile timer service
 * when the instance is initialized it registers its own handler
 * at its instance of the EveryTimer
 * This handler calls the ISR_Timer.run() to handle the services
 * Services are regsitered directly at the exposed instance of the
 * ISR_Timer
 *
 * 2020-05-09: AW Rev A
 *
 *-----------------------------------------------*/

//--- includes

#include <ISR_Timer.h>
#include "Arduino.h"

//--- definitions

const unsigned int defaultPeriodms = 10;
const unsigned int usPerms = 1000;


class MCTimer
{
	public:
		ISR_Timer TimerService;
		
		//constructor can be called with a different period time
		MCTimer();
		MCTimer(unsigned long period);
	
	private:
		unsigned long HwPeriod = usPerms*defaultPeriodms;
		void init(void);
};
				

#endif
