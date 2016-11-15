/*
 * usb.cpp
 *
 *  Created on: 27 Sep 2016
 *      Author: Michael Drummond
 */

#include <iostream>
#include <sys/mount.h>
#include <errno.h>
#include <string.h>
#include "usb.h"


int usb::mountUSB() {

	if (mount(SOURCEADDR, TARGETADDR, FILESYSTEM, NULL, NULL)) {
		if (errno == EBUSY) {
			std::cout << "Mount point busy" << std::endl;
			return 0;
		} else {
			std::cout << "Mount error: " << strerror(errno) << std::endl;
			return -1;
		}
	} else {
		std::cout << "Mount successful" << std::endl;
		return 1;
	}

}

int usb::unmountUSB() {

	if (umount(TARGETADDR)) {
		std::cout << "Unmount error: " << strerror(errno) << std::endl;
		return 0;
	} else {
		std::cout << "Unmount successful" << std::endl;
		return 1;
	}

}
