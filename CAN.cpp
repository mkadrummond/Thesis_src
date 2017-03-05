/*
 * CAN.cpp
 *
 *  Created on: 4 Aug 2016
 *      Author: mkadrummond
 */

#include "CAN.h"
#include<iostream>
#include <sys/ioctl.h>

#define SIOCSCANBAUDRATE	(SIOCDEVPRIVATE+0)

using namespace std;

CANSocket::CANSocket() {
	soc = NULL;
	read_can_port = NULL;
}

CANSocket::CANSocket(const char *port) {
	open_port(port);
}

CANSocket::~CANSocket() {}

// Member functions
int CANSocket::open_port(const char *port)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    CANSocket::soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0)
    {
        return (-1);
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        return (-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {

        return (-1);
    }

    return 0;
}

int CANSocket::initNode(__u8 nodeID) {

	struct can_frame nodeStop;
		nodeStop.can_id = 0x700;
		nodeStop.can_dlc = 2;
		nodeStop.data[0] = 0x02;
		nodeStop.data[1] = 0x00;
	if (send_port(&nodeStop) < 0) {
		return -1;
	}

	struct can_frame setNodeID;
		setNodeID.can_id = 0x7E7;
		setNodeID.can_dlc = 6;
		setNodeID.data[0] = 0x02;
		setNodeID.data[1] = 0x00;
		setNodeID.data[2] = 0x00;
		setNodeID.data[3] = 0x12;
		setNodeID.data[4] = 0x91;
		setNodeID.data[5] = nodeID;
	if (send_port(&setNodeID) < 0) {
		return -1;
	}

	struct can_frame nodeStart;
		nodeStart.can_id = 0x700;
		nodeStart.can_dlc = 2;
		nodeStart.data[0] = 0x01;
		nodeStart.data[1] = 0x01;
	if (send_port(&nodeStart) < 0) {
		return -1;
	}

	return 0;
}

int CANSocket::send_port(struct can_frame *frame)
{
    int retval;
    retval = write(this->soc, frame, sizeof(struct can_frame));
    if (retval != sizeof(struct can_frame))
    {
        return (-1);
    }
    else
    {
        return (0);
    }
}

void CANSocket::read_port()
{
    struct can_frame frame_rd;
    int recvbytes = 0;

    int read_can_port = 1;
    while(read_can_port)
    {
        struct timeval timeout = {1, 0};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(soc, &readSet);

        if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
        {
            if (!read_can_port)
            {
                break;
            }
            if (FD_ISSET(soc, &readSet))
            {
            	// To make this work two lines were changed, based on the information here:
            	// https://chemnitzer.linux-tage.de/2012/vortraege/folien/1044_SocketCAN.pdf
                //recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
            	recvbytes = read(soc, &frame_rd, sizeof(frame_rd));
                if(recvbytes)
                {
                    //printf("dlc = %d, data = %s\n", frame_rd.can_dlc,frame_rd.data);
                	printf("ID = 0x%X DLC = %d ", frame_rd.can_id, frame_rd.can_dlc);
                	printf("data = ");
                	for(int i = 0; i < frame_rd.can_dlc; i++) {
                		printf("0x%X ", frame_rd.data[i]);
                	}
                	printf("\n");
                }
            }
        }

    }

}

int CANSocket::close_port()
{
    close(soc);
    return 0;
}

int CANSocket::sendFrame(canid_t can_id, __u16 data) {

	can_frame frame;
	frame.can_id = can_id;
	frame.can_dlc = 3;

	frame.data[0] = data >> 8;
	frame.data[1] = data & 0xFF;
	frame.data[2] = 0;

	if (send_port(&frame) < 0) {
		return -1;
	}

	return 0;
}



