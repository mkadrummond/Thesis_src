/*
 * CAN.h
 *
 *  Created on: 4 Aug 2016
 *      Author: mkadrummond
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>


#include <unistd.h>

class CANSocket;

class CANSocket {


public:
	CANSocket();
	CANSocket(const char *port);

	// Functions
	int open_port(const char *);
	int send_port(struct can_frame *);
	void read_port();
	int close_port();
	int initNode(__u8);
	int sendFrame(canid_t, __u16);

	~CANSocket();

private:
	int soc;
	int read_can_port;

};









#endif /* CAN_H_ */
