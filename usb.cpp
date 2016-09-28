/*
 * usb.cpp
 *
 *  Created on: 27 Sep 2016
 *      Author: mkadrummond
 */

#include <stdlib.h>
#include <iostream>
#include <sys/mount.h>
#include <errno.h>
#include <string.h>
#include "usb.h"



int usb::mountUSB() {

	if (mount("/dev/sda1", "/mnt/usb", "vfat", NULL, NULL)) {
		if (errno == EBUSY) {
			std::cout << "Mountpoint busy" << std::endl;
			return -1;
		} else {
			std::cout << "Mount error: " << strerror(errno) << std::endl;
			return -2;
		}
	} else {
		std::cout << "Mount successful" << std::endl;
		return 0;
	}

}

int usb::unmountUSB() {

	if (umount("/mnt/usb")) {
		std::cout << "Unmount error: " << strerror(errno) << std::endl;
		return -1;
	} else {
		std::cout << "Unmount successful" << std::endl;
		return 0;
	}

}
