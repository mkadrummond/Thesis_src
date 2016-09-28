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

void checkResults(int);
int showSchedParam(pthread_t);
int startThread(void *(*) (void *), float*);

#endif /* THREAD_H_ */
