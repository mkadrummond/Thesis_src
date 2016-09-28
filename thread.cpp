/*
 * thread.cpp
 *
 *  Created on: 8 Sep 2016
 *      Author: mkadrummond
 */

#include "thread.h"
#include <cstring>
#include <iostream>
#include <unistd.h>
using namespace std;

pthread_t          	thread;
struct sched_param 	param;
int 				rc;
int 				policy;

void checkResults(std::string str, int val) {
	if (val) {
		cout << str << val << endl;
	}
}

int showSchedParam(pthread_t thread) {
	printf("Get scheduling parameters\n");
	rc = pthread_getschedparam(thread, &policy, &param);
	checkResults("pthread_getschedparam", rc);

	printf("The thread scheduling parameters indicate:\n"
			"priority = %d\n", param.sched_priority);
	return param.sched_priority;
}


//int startThread(void *(*threadfunc) (void *)) {
int startThread(void *(*threadfunc) (void *), float *arg) {

	rc = 0;
	policy = SCHED_RR; 	// Round Robin

	//rc = pthread_create(&thread, NULL, threadfunc, NULL);
	rc = pthread_create(&thread, NULL, threadfunc, arg);
	checkResults("pthread_create", rc);

	usleep(100000);

	std::memset(&param, 0, sizeof(param));

	param.sched_priority = THREAD_PRIORITY;
	rc = pthread_setschedparam(thread, policy, &param);
	checkResults("pthread_setschedparam", rc);

	return 0;
}
