/*
 * DeviceControl.cpp
 *
 *  Created on: 15 Nov 2016
 *      Author: mkadrummond
 */

#include<iostream>

#include "pid.h"
#include "CAN.h"
#include "GPIO.h"
#include "usb.h"

#include <cstring>

#include "deviceControl.h"
#include "prussdrv.h"
#include "pruss_intc_mapping.h"


// Includes for shutting down
#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include<fstream>
#include<sstream>

#include <cmath>

// Includes for the display and I2C interface
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <string>

DeviceControl::DeviceControl(CANSocket *can)
	:	pressureSetPoint(0),
		pressureMeasured(0),
		pressureInBinary(0),
		exitThread(0),
		can(can)
		{

		// Initialise the GPIO pins for the Analogue switch that
		// selects the voltage divider over which the force is read
		switch1GPIO = new exploringBB::GPIO(112); 	// Connects AIN1
		switch2GPIO = new exploringBB::GPIO(111);	// Connects AIN2
		switch3GPIO = new exploringBB::GPIO(110);	// Connects AIN3

		switch1GPIO->setDirection(exploringBB::OUTPUT);
		switch2GPIO->setDirection(exploringBB::OUTPUT);
		switch3GPIO->setDirection(exploringBB::OUTPUT);

		switch1GPIO->streamOpen();
		switch2GPIO->streamOpen();
		switch3GPIO->streamOpen();

		}

void *DeviceControl::controlfunc(void) {

	float pressureOutput;

	float dt = 0.01;
    double max = 1;
    double min = -1;
    float Kp = 0.7;
    float Ki = 0.2;
    float Kd = 0;

    PID *pidControl = new PID(dt, max, min, Kp, Kd, Ki);
    std::cout << "PID controller instantiated" << std::endl;


	this->exitThread = 0;

	while (this->exitThread == 0) {

		// Wait for event completion (interrupt) from PRU
		prussdrv_pru_wait_event (PRU_EVTOUT_0);

		// Reset PRU interrupt
		prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);


		int pressure = readAnalog(0);

 		this->pressureMeasured = binary2Bar(pressure);

 		pressureOutput = pidControl->calculate( this->pressureSetPoint*1000, this->pressureMeasured );

		this->can->sendFrame(0x100, pressureOutput);

	}

	std::cout << "Thread closed" << std::endl;
	this->exitThread = 0;

	return 0;
}

void *DeviceControl::thread_helper(void *context) {
        return ((DeviceControl *)context)->controlfunc();
    }

int DeviceControl::startThread(DeviceControl *control) {

	pthread_t thread;
	struct sched_param param;
	int policy = SCHED_RR; 	// Round Robin

	if ( pthread_create(&thread, NULL, &DeviceControl::thread_helper, control) ) {
		perror("pthread_create");
	}

	usleep(100000);

	std::memset(&param, 0, sizeof(param));

	param.sched_priority = THREAD_PRIORITY;
	if ( pthread_setschedparam(thread, policy, &param ) ) {
		perror("pthread_setschedparam");
	}

	if ( pthread_getschedparam(thread, &policy, &param ) ) {
		perror("pthread_getschedparam");
	}

	std::cout << "The thread scheduling parameters indicate: "
			"priority = " << param.sched_priority << std::endl;

	return 0;
}

int DeviceControl::terminateThread() {

	this->exitThread = 1;

	return 0;
}

int DeviceControl::initCANbus() {

	// The bit rate is fixed at 1MHz
	int retv = system("ip link set can0 up type can bitrate 1000000");
	if (retv == -1) {
		std::cout << "Error: CAN bus not initialised" << std::endl;
	} else {
		if (WEXITSTATUS(retv) == 127) {
			std::cout << "Error: could not execute /bin/sh" << std::endl;
		} else {
			std::cout << "CAN bus initialised. System returned with " << WEXITSTATUS(retv) << std::endl;

			this->can = new CANSocket("can0");

			// Initialise the valve
			int ret = this->can->initNode(0x01);
			if (ret < 0) {
				perror("CANbus");
			}
		}
	}

	return retv;
}

int DeviceControl::terminateCANbus() {

	int retval = this->can->close_port();
	system("ifconfig can0 down");

	return retval;
}

int DeviceControl::initValve() {

	int retval = this->can->initNode(0x01);

	if (retval < 0) {
		perror("Valve initialisation");
	}

	return retval;
}

int DeviceControl::initI2C() {
	int r;
	int fd;

	// Setup i2c
	char *dev = "/dev/i2c-2";

	fd = open(dev, O_RDWR );
	if(fd < 0) {
		perror("Opening i2c device node\n");
	}

	int addr = ADDRESS; // I2C address of the LCD
	r = ioctl(fd, I2C_SLAVE, addr);
	if(r < 0) {
		perror("Selecting i2c device\n");
	}

	return fd;
}

int DeviceControl::initPRU() {
	// Initialise structure used by prussdrv_pruintc_intc
	// PRUSS_INTC_INITDATA is found in pruss_intc_mapping.h
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	// Allocate and initialise memory
	prussdrv_init ();
	prussdrv_open (PRU_EVTOUT_0);

	// Map PRU's interrupts
	prussdrv_pruintc_init(&pruss_intc_initdata);

	// Load and execute the PRU program on the PRU
	// Note: I only got this to work with this absolute file path!
	prussdrv_exec_program (PRU_NUM, PRU_BIN_ADDR);

	return 0;
}

int DeviceControl::terminatePRU() {

	prussdrv_exit ();

	return 0;
}

int DeviceControl::readAnalog(int number){
	std::stringstream ss;
    ss << LDR_PATH << number << "_raw";
    std::fstream fs;
    fs.open(ss.str().c_str(), std::fstream::in);
    fs >> number;
    fs.close();
    return number;
}

int DeviceControl::analogSwitch(int selector) {

	switch (selector) {
	case 0:
	case 1:
		control->switch1GPIO->streamWrite(exploringBB::LOW);
		control->switch2GPIO->streamWrite(exploringBB::HIGH);
		control->switch3GPIO->streamWrite(exploringBB::LOW);
		break;
	case 2:
		control->switch1GPIO->streamWrite(exploringBB::LOW);
		control->switch2GPIO->streamWrite(exploringBB::LOW);
		control->switch3GPIO->streamWrite(exploringBB::HIGH);
		break;

	case 3:
		control->switch1GPIO->streamWrite(exploringBB::HIGH);
		control->switch2GPIO->streamWrite(exploringBB::LOW);
		control->switch3GPIO->streamWrite(exploringBB::LOW);
	}

	return 0;
}

float DeviceControl::measureForce(rig::muscleSize choiceOfMuscle) {

	float force;

	switch(choiceOfMuscle) {
	// The 5mm and 10mm muscles use the same voltage divider and
	// are read from the same ADC input pin
	case rig::muscle5mm:
	case rig::muscle10mm:
		force = this->readAnalog(MAX3V3);			// read the ADC value
		force = binary2Newton(force, 0);
		break;
	case rig::muscle20mm:
		force = this->readAnalog(MAX6V);
		force = binary2Newton(force, 1);
		break;
	case rig::muscle40mm:
		force = this->readAnalog(MAX10V);
		force = binary2Newton(force, 2);
		break;
	default:
		force = 999;
		return -1;
	}

	return force;
}

float DeviceControl::binary2Newton(float force, int choiceOfMuscle) {

	force = force * 1800/(pow(2,12)-1); // convert to milli-volts

	switch(choiceOfMuscle) {
	case 0:
		force = force * (47.5 + 57)/57; // voltage divider
		break;
	case 1:
		force = force * (77.2 + 33.2)/33.2;
		break;
	case 2:
		force = force * (91+20)/20;
		break;
	default:
		perror("DeviceControl::binary2Newton()");
		force = 0;
	}

	force = force * 327/1000; 	// force sensor ratio

	return force;
}

float DeviceControl::binary2Bar(int adcValue) {
	// The read value into millivolts: *1800/(2^12-1)
	// voltage divider: *(91+51)/51
	// The two together: (1800/(2^12-1))*(91+51)/51 = 1.224
	float pressureAsVoltage = adcValue*1.2238742; 	//

	// Linear interpolation of values given in the pressure sensor data sheet:
	// 0bar -> 0.2V
	// 10bar -> 4.7V
	// The return value is in mbar
	return (pressureAsVoltage-200)*10000/(4700-200);
}

int DeviceControl::writePressurePointsToUSB() {

	usb::mountUSB();

	// Open a file stream to the USB stick
	ofstream datafile;
	datafile.open("/mnt/usb/data.txt", std::ios::out | std::ios::app);

	// Fill a line of data.txt with all the pressure values
	for (int k = 0; k < (int) control->vMeasurements.size(); k++) {
		datafile << "\t" <<  control->vMeasurements[k]->pressure;
	}
	datafile << "\n"; 	// End the line

	datafile.close();

	usb::unmountUSB();

	return 0;
}

int DeviceControl::writeMeasurementsToUSB(int index) {

	usb::mountUSB();

	// Open a file stream to the USB stick
	ofstream datafile;
	datafile.open("/mnt/usb/data.txt", std::ios::out | std::ios::app);

	datafile <<  control->vLengths[index]->length << ":\t";
	for (int k = 0; k < (int) control->vMeasurements.size(); k++) {
		datafile <<  static_cast<int>(control->vMeasurements[k]->force) << "\t";
	}
	datafile << "\n";

	datafile.close();

	usb::unmountUSB();

	return 0;
}

int DeviceControl::saveProgram() {

	int retval;

	ofstream programFile;
	programFile.open("/mnt/usb/program.txt", std::ios::out | std::ios::trunc);
	if (programFile.is_open()) {

		// Total number of Lengths; helpful for when we want to load the program later:
		programFile << control->vLengths.size() << "\n";

		for(unsigned int i = 0; i < control->vLengths.size(); i++) {
			programFile << control->vLengths[i]->length << "\t";
			programFile << control->vLengths[i]->pressure << "\t\n";
		}

		programFile.close();
		retval = 0;

	} else {
		perror("program.txt");
		retval = 1;
	}
	return retval;
}

int DeviceControl::loadProgram() {

	std::ifstream programFile("/mnt/usb/program.txt", std::ios::in);

	int n;
	float pressure;
	int length;
	int retval;

	if (programFile >> n) {

		// Delete any previously stored lengths
		this->vLengths.clear();


		while (n > 0) {
			if (programFile >> length >> pressure) {
				this->vLengths.push_back(new rig::Length);
				this->vLengths.back()->pressure = pressure;
				this->vLengths.back()->length = length;

			} else {
				perror("program.txt");
				return 1;
			}
			n--;
		}

		programFile.close();
		retval = 0;

	} else {
		perror("program.txt");
		retval = 1;
	}

	return retval;
}
