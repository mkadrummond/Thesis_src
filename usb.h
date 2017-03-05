/*
 * usb.h
 *
 *  Created on: 27 Sep 2016
 *      Author: Michael Drummond
 */

#ifndef USB_H_
#define USB_H_

#define	SOURCEADDR 	"/dev/sda1"
#define TARGETADDR 	"/mnt/usb"
#define FILESYSTEM	"vfat"

namespace usb {

int mountUSB();
int unmountUSB();

}

#endif /* USB_H_ */
