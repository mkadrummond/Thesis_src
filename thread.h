/*
 * thread.h
 *
 *  Created on: 8 Sep 2016
 *      Author: mkadrummond
 */

#ifndef THREAD_H_
#define THREAD_H_

#include <pthread.h>
#include <sched.h>

#define THREAD_PRIORITY 50

#if 0
//PRU
#include <stdlib.h>
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#define PRU_NUM 0   	// using PRU0 for the timer
#define PRU_BIN_ADDR 	"/home/mkadrummond/pru/timer.bin"	//

// PID controller
// from: https://gist.github.com/bradley219/5373998
#include "pid.h"

#include "adc.h"

class Thread;

class Thread {
public:
	Thread();

	bool StartInternalThread();

	float pressureSP;
	double pressureInBinary;
	double pressureInBar;

	ADC *adc;

	static int startThread(void *(*) (void *), float*);

	int showSchedParam(pthread_t);

	//typedef void FPTR(void *);
	//static FPTR controlFunc;


	~controlThread();

private:


};
#endif

void checkResults(int);
int startThread(void *(*) (void *), float*);
int showSchedParam(pthread_t);
//void *controlFunc(void *);


#endif /* THREAD_H_ */
