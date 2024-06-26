/****************************************************************************************************************************
 * ISR_Timer.cpp
 * For Arduino boards (UNO, Nano, Mega, etc. )
 * Written by Khoi Hoang
 * 
 * Built by Khoi Hoang https://github.com/khoih-prog/TimerInterrupt
 * Licensed under MIT license
 * Version: v1.0.2
 * 
 * Timer0: Timer0 is a 8bit timer. In the Arduino world timer0 is been used for the timer functions, like delay(), millis() 
 * and micros(). 
 * If you change Timer0 registers, this may influence the Arduino timer function. So you should know what you are doing.
 * Timer1: Timer1 is a 16bit timer. In the Arduino world the Servo library uses timer1 on Arduino Uno (Timer5 on Arduino Mega).
 * Timer2: Timer2 is a 8bit timer like Timer0. In the Arduino work the tone() function uses Timer2.
 * Timer3, Timer4, Timer5: Timer 3,4,5 are only available on Arduino Mega boards. These timers are all 16bit timers.
 * 
 * Now we can use these new 16 ISR-based timers, while consuming only 1 hardware Timer.
 * Their independently-selected, maximum interval is practically unlimited (limited only by unsigned long miliseconds)
 * The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
 * Therefore, their executions are not blocked by bad-behaving functions / tasks.
 * This important feature is absolutely necessary for mission-critical tasks.
 *
 * Based on SimpleTimer - A timer library for Arduino.
 * Author: mromani@ottotecnica.com
 * Copyright (c) 2010 OTTOTECNICA Italy
 * 
 * Based on BlynkTimer.h
 * Author: Volodymyr Shymanskyy
 *
 * Version Modified By   Date      Comments
 * ------- -----------  ---------- -----------
 *  1.0.0   K Hoang      23/11/2019 Initial coding
 *  1.0.1   K Hoang      25/11/2019 New release fixing compiler error
 *  1.0.2   K.Hoang      28/11/2019 Permit up to 16 super-long-time, super-accurate ISR-based timers to avoid being blocked
*****************************************************************************************************************************/

#include "ISR_Timer.h"
#include <string.h>

// Select time function:
//static inline unsigned long elapsed() { return micros(); }
static inline unsigned long elapsed() { return millis(); }


ISR_Timer::ISR_Timer()
    : numTimers (-1)
{
}

void ISR_Timer::init() {
    unsigned long current_millis =millis();    //elapsed();

    for (int i = 0; i < MAX_TIMERS; i++) {
        memset((void*) &timer[i], 0, sizeof (timer_t));
        timer[i].prev_millis = current_millis;
    }

    numTimers = 0;
}


void ISR_Timer::run() {
    int i;
    unsigned long current_millis;

    // get current time
    current_millis = millis();   //elapsed();

    for (i = 0; i < MAX_TIMERS; i++) {

        timer[i].toBeCalled = DEFCALL_DONTRUN;

        // no callback == no timer, i.e. jump over empty slots
        if (timer[i].callback != NULL) {

            // is it time to process this timer ?
            // see http://arduino.cc/forum/index.php/topic,124048.msg932592.html#msg932592

            if ((current_millis - timer[i].prev_millis) >= timer[i].delay) {

                unsigned long skipTimes = (current_millis - timer[i].prev_millis) / timer[i].delay;
                // update time
                timer[i].prev_millis += timer[i].delay * skipTimes;

                // check if the timer callback has to be executed
                if (timer[i].enabled) {

                    // "run forever" timers must always be executed
                    if (timer[i].maxNumRuns == RUN_FOREVER) {
                        timer[i].toBeCalled = DEFCALL_RUNONLY;
                    }
                    // other timers get executed the specified number of times
                    else if (timer[i].numRuns < timer[i].maxNumRuns) {
                        timer[i].toBeCalled = DEFCALL_RUNONLY;
                        timer[i].numRuns++;

                        // after the last run, delete the timer
                        if (timer[i].numRuns >= timer[i].maxNumRuns) {
                            timer[i].toBeCalled = DEFCALL_RUNANDDEL;
                        }
                    }
                }
            }
        }
    }

    for (i = 0; i < MAX_TIMERS; i++) {
        if (timer[i].toBeCalled == DEFCALL_DONTRUN)
            continue;

        if (timer[i].hasParam)
            (*(timer_callback_p)timer[i].callback)(timer[i].param);
        else
            (*(timer_callback)timer[i].callback)();

        if (timer[i].toBeCalled == DEFCALL_RUNANDDEL)
            deleteTimer(i);
    }
}


// find the first available slot
// return -1 if none found
int ISR_Timer::findFirstFreeSlot() {
    // all slots are used
    if (numTimers >= MAX_TIMERS) {
        return -1;
    }

    // return the first slot with no callback (i.e. free)
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (timer[i].callback == NULL) {
            return i;
        }
    }

    // no free slots found
    return -1;
}


int ISR_Timer::setupTimer(unsigned long d, void* f, void* p, bool h, unsigned n) {
    int freeTimer;

    if (numTimers < 0) {
        init();
    }

    freeTimer = findFirstFreeSlot();
    if (freeTimer < 0) {
        return -1;
    }

    if (f == NULL) {
        return -1;
    }

    timer[freeTimer].delay = d;
    timer[freeTimer].callback = f;
    timer[freeTimer].param = p;
    timer[freeTimer].hasParam = h;
    timer[freeTimer].maxNumRuns = n;
    timer[freeTimer].enabled = true;
    timer[freeTimer].prev_millis = elapsed();

    numTimers++;

    return freeTimer;
}


int ISR_Timer::setTimer(unsigned long d, timer_callback f, unsigned n) {
  return setupTimer(d, (void *)f, NULL, false, n);
}

int ISR_Timer::setTimer(unsigned long d, timer_callback_p f, void* p, unsigned n) {
  return setupTimer(d, (void *)f, p, true, n);
}

int ISR_Timer::setInterval(unsigned long d, timer_callback f) {
    return setupTimer(d, (void *)f, NULL, false, RUN_FOREVER);
}

int ISR_Timer::setInterval(unsigned long d, timer_callback_p f, void* p) {
  return setupTimer(d, (void *)f, p, true, RUN_FOREVER);
}

int ISR_Timer::setTimeout(unsigned long d, timer_callback f) {
    return setupTimer(d, (void *)f, NULL, false, RUN_ONCE);
}

int ISR_Timer::setTimeout(unsigned long d, timer_callback_p f, void* p) {
  return setupTimer(d, (void *)f, p, true, RUN_ONCE);
}

bool ISR_Timer::changeInterval(unsigned numTimer, unsigned long d) {
    if (numTimer >= MAX_TIMERS) {
        return false;
    }

    // Updates interval of existing specified timer
    if (timer[numTimer].callback != NULL) {
        timer[numTimer].delay = d;
        timer[numTimer].prev_millis = elapsed();
        return true;
    }
    // false return for non-used numTimer, no callback
    return false;
}

void ISR_Timer::deleteTimer(unsigned timerId) {
    //don't delete timers which are invalid
    if (timerId >= MAX_TIMERS) {
        return;
    }

    //don't delete timers which are invalid
    if (timerId < 0) {
        return;
    }

    // nothing to delete if no timers are in use
    if (numTimers == 0) {
        return;
    }

    // don't decrease the number of timers if the
    // specified slot is already empty
    if (timer[timerId].callback != NULL) {
        memset((void*) &timer[timerId], 0, sizeof (timer_t));
        timer[timerId].prev_millis = elapsed();

        // update number of timers
        numTimers--;
    }
}


// function contributed by code@rowansimms.com
void ISR_Timer::restartTimer(unsigned timerId) {
    if (timerId >= MAX_TIMERS) {
        return;
    }

    //don't delete timers which are invalid
    if (timerId < 0) {
        return;
    }

    timer[timerId].prev_millis = elapsed();
}


bool ISR_Timer::isEnabled(unsigned numTimer) {
    if (numTimer >= MAX_TIMERS) {
        return false;
    }

    return timer[numTimer].enabled;
}


void ISR_Timer::enable(unsigned numTimer) {
    if (numTimer >= MAX_TIMERS) {
        return;
    }

    timer[numTimer].enabled = true;
}


void ISR_Timer::disable(unsigned numTimer) {
    if (numTimer >= MAX_TIMERS) {
        return;
    }

    timer[numTimer].enabled = false;
}

void ISR_Timer::enableAll() {
    // Enable all timers with a callback assigned (used)
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (timer[i].callback != NULL && timer[i].numRuns == RUN_FOREVER) {
            timer[i].enabled = true;
        }
    }
}

void ISR_Timer::disableAll() {
    // Disable all timers with a callback assigned (used)
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (timer[i].callback != NULL && timer[i].numRuns == RUN_FOREVER) {
            timer[i].enabled = false;
        }
    }
}

void ISR_Timer::toggle(unsigned numTimer) {
    if (numTimer >= MAX_TIMERS) {
        return;
    }

    timer[numTimer].enabled = !timer[numTimer].enabled;
}


unsigned ISR_Timer::getNumTimers() {
    return numTimers;
}
