/*--------------------------------------------------------------
 * MCTimer.cpp
 * Implementation of the MCTimer behavior
 *
 * 2020-05-09 AW Rev A
 *
 *--------------------------------------------------------------*/
 
 //---includes
 
 #include <MCTimer.h>
 
#ifdef ARDUINO_ARCH_MEGAAVR
#include <EveryTimerB.h>
#define Timer1 TimerB2    // use TimerB2 as a drop in replacement for Timer1
#else // assume architecture supported by TimerOne ....
#include "TimerOne.h"
#endif

 //---globals

MCTimer OsTimer;
		
static void Handler(void);

 
 //---implementaton
 
 MCTimer::MCTimer()
 {
    init();
 }
 
 MCTimer::MCTimer(unsigned long period)
 {
	HwPeriod = usPerms * period;
    init(); 
 }
 
 static void Handler()
 {
    OsTimer.TimerService.run();
 }
 
 /* ----------------------------------------------------
  * MCTimer::init()
  * initialize the low leven hw timer and regsiter the
  * Handler as the low leven int handler method
  * 
  * 2020-05-10 AW Rev A
  * 
  * ----------------------------------------------------*/
  
 void MCTimer::init()
 {
    // initialize the HwTimer
    // 1) basic init
    // 2) stop service
    // 3) attach handler
    // 4) set period in µs - which also starts the serivce
    Timer1.initialize();
    Timer1.stop();
    Timer1.attachInterrupt(Handler,HwPeriod);
 }
