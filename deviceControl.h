/*
 * rigControl.h
 *
 *  Created on: 15 Nov 2016
 *      Author: mkadrummond
 */

#ifndef DEVICECONTROL_H_
#define DEVICECONTROL_H_

#include "CAN.h"
#include "GPIO.h"
#include "pid.h"
#include <pthread.h>
#include <sched.h>
#include <vector>

// Linux path to read the ADC input from the force sensor
#define LDR_PATH "/sys/bus/iio/devices/iio:device0/in_voltage"

// LCD Address
#define ADDRESS 0x27

#define THREAD_PRIORITY 50

#define PRU_NUM 0   	// using PRU0 for the timer
#define PRU_BIN_ADDR 	"/home/mkadrummond/pru/timer.bin"

#define MAX10V	1
#define MAX3V3 	2
#define MAX6V	3

namespace rig {
	enum muscleSize { muscle5mm, muscle10mm, muscle20mm, muscle40mm};

	struct Length {
		int length;
		float pressure;
	};

	struct Measurement {
		int length;
		float pressure;
		int force;
	};
}

/**
 * @class DeviceControl
 * @brief Controls the back-end functions of the test rig
 *
 * DeviceControl controls the back-end of the test rig. It is responsible for
 * initialising the various elements of the control system (CAN bus, the
 * pressure valve, the I2C bus, and the Programmable Real-time units); starting
 * the thread that runs the PID controller in the background; reading pressure
 * and force measurements; and storing the data.
 */
class DeviceControl {
public:

	/**
	 * @brief Constructor that defines CAN bus and GPIO pins
	 *
	 * This constructor is passed a pointer to CAN object with which it will
	 * communicate with the valve. In addition, it defines the GPIO pins to
	 * be used with the analogue switch that chooses the voltage divider over
	 * which the force measurement will be taken.
	 *
	 * @param can Pointer to CANSocket object for CAN communication
	 */
	DeviceControl(CANSocket *can);

	float pressureSetPoint;		///< Pressure set point in bar
	double pressureMeasured;	///< Measured pressure in bar

	std::vector<rig::Length*> vLengths;			  ///< Stores Length data
	std::vector<rig::Measurement*> vMeasurements; ///< Stores measurement data

	/**
	 * @brief Function pointer to the control function
	 *
	 * This function pointer is passed to a thread running parallel to the main
	 * thread. It runs in a loop that waits in every cycle for an interrupt
	 * from the PRU timer. Then takes the measured pressure and set point to
	 * calculate an output value with the help of a PID controller.
	 */
	void *controlfunc(void);

	/**
	 * @brief A helper function to get the thread started with controlfunc()
	 *
	 * This function is used in startThread() to get a thread started
	 * with the function controlfunc(). In startThread(), thread_helper() is
	 * passed as a parameter of pthread_create. pthread_create is a C function
	 * and does not understand the context ("this") of a function it is
	 * passed. This helper function circumvents this problem by itself being static
	 * while still calling the appropriate controlfunc().
	 *
	 * @param context The context of the function
	 */
	static void *thread_helper(void *context);

	/**
	 * @brief Starts a thread in the DeviceControl object
	 *
	 * Starts a thread with a round robin scheduling policy and a priority as
	 * defined in THREAD_PRIORITY
	 *
	 * @param control The deviceControl object to which the thread will belong
	 * @return An integer, 0 is success
	 */
	int startThread(DeviceControl *control);

	/**
	 * @brief Terminates the thread
	 *
	 * Terminates the control loop thread that DeviceControl is running
	 *
	 * @return An integer, 0 is success
	 */
	int terminateThread();

	/**
	 * @brief Initialises the CAN bus and the Festo valve
	 *
	 * Enables a network interface to the CAN bus and binds the CAN socket of
	 * CAN object of DeviceControl. If successful the pressure valve will be
	 * initialised according to the data sheet with the Node ID
	 * 0x01.
	 *
	 * @return An integer: 	0 = success,
	 *  				   -1 = network interface failed,
	 *  					2 = network interface busy (most likely already enabled)
	 */
	int initCANbus();

	/**
	 * @brief Terminates the CAN bus
	 *
	 * Terminates the CAN socket and closes the network interface to the CAN
	 * bus.
	 *
	 * @return An integer: 0 is success
	 */
	int terminateCANbus();

	/**
	 * @brief Initialises the pressure valve
	 *
	 * Initialises the pressure valve by putting a series of commands on the
	 * CAN bus that sets the valve's Node ID to 0x01 and prepares it to receive
	 * valve position values.
	 *
	 * @return An integer: 0 is success
	 */
	int initValve();

	/**
	 * @brief Initialises an I2C bus
	 *
	 * Initialises the I2C2 bus for communication with the LCD. This is done by
	 * creating and returning a file descriptor to /dev/i2c-2 and initialising
	 * communication by sending the address of the LCD.
	 *
	 * @return The file descriptor of the I2C bus
	 */
	int initI2C();

	/**
	 * @brief Initialise the Programmable Real-Time Units
	 *
	 * Allocates and initialises memory for the Programmable Real-Time units;
	 * maps the PRU's interrupts; loads and executes the assembler program on
	 * the BeagleBone.
	 *
	 * @return An integer: 0 is success
	 */
	int initPRU();

	/**
	 * @brief Terminates the program running on the PRU
	 *
	 * Exits the program running on the Programmable Real-Time Unit(s) and
	 * closes the memory mapping to the main processor
	 *
	 * @return An integer: 0 is success
	 */
	int terminatePRU();

	/**
	 * @brief Reads an ADC channel
	 *
	 * Opens a file stream to the appropriate ADC channel input and reads the
	 * calculated value.
	 *
	 * @param channel The ADC channel to be read
	 * @return An integer: the measured value at the ADC input (0-4095)
	 */
	int readAnalog(int channel);

	/**
	 * @brief Selects the force measurement channel
	 *
	 * Selects the voltage divider channel over which the force measurement
	 * signal reaches the ADC input pin. This happens with the appropriate
	 * switching of the DeviceControl GPIO pins which control an analogue
	 * switch.
	 *
	 * @param selector The desired channel (0-3)
	 * @return An integer: 0 is success
	 */
	int analogSwitch(int selector);

	/**
	 * @brief Reads the ADC and converts the value to Newtons
	 *
	 * Uses the choiceOfMuscle parameter to select the correct ADC input
	 * channel. Reads the input and converts it to a value in Newtons of force.
	 *
	 * @param choiceOfMuscle
	 * @return A float: the measured force in Newton
	 */
	float measureForce(rig::muscleSize choiceOfMuscle);

	/**
	 * @brief Converts an ADC measurement to force in Newtons
	 *
	 * Converts a BeagleBone Black ADC measurement into Newtons of force,
	 * depending on the specific muscle (and therefore voltage divider)settings
	 * are used.
	 *
	 * @param force A value read directly from the BBB ADC (0-4095)
	 * @param choiceOfMuscle The muscle diameter setting, determines the
	 * 						 voltage divider settings
	 * @return A float: the force in Newtons
	 */
	float binary2Newton(float force, int choiceOfMuscle);

	/**
	 * @brief Converts an ADC measurement to pressure in milli-bar
	 *
	 * Converts a BeagleBone Black ADC measurement into milli-bars of pressure.
	 * Uses voltage divider settings based on the hardware, and a linear
	 * interpolation of the pressure sensor characteristic.
	 *
	 * @param adcValue A value read directly from the BBB ADC (0-4095)
	 * @return A float: pressure in milli-bar
	 */
	float binary2Bar(int adcValue);

	/**
	 * @brief Writes the pressure points used in a measurement to the USB stick
	 *
	 * Mounts the USB stick and opens a filestream to a file called data.txt on
	 * the stick. The file is opened in "append" mode, such that existing data
	 * in the file is not erased. The pressure values of the Measurement structs in
	 * vMeasurements are transfered to a single line in this file, separated by
	 * tabs. The line ends with a line break, the file is closed and the USB
	 * stick us unmounted.
	 *
	 * @return An integer: success is o
	 */
	int writePressurePointsToUSB();

	/**
	 * @brief Writes a row of measurements to the USB stick
	 *
	 * Opens a filestream to a file called data.txt on
	 * the stick. The file is opened in "append" mode, such that existing data
	 * in the file is not erased. Writes the length value from the Length
	 * struct in the vLength vector that the index parameter points to to the
	 * file. This is followed by (on the same line, separated by a tab) the
	 * measurement values of the Measurement structs in vMeasurements. The line
	 * ends with a line break and the file is closed.
	 *
	 * @param index An index of which length (from vLengths) has been measured
	 * @return An integer: success is 0
	 */
	int writeMeasurementsToUSB(int index);

	/**
	 * @brief Stores details of the program on the USB stick
	 *
	 * Opens a filesstream to a file called program.txt on the USB stick. The
	 * file is opened in "truncate" mode, such that any previously stored data
	 * is deleted when it is opened. The values from the Length structs in
	 * vLengths is written to the file. The filestream is closed.
	 *
	 * @return An integer: 0 is success
	 */
	int saveProgram();

	/**
	 * @brief Loads details of a program from the USB stick
	 *
	 * Opens a filesstream to a file called program.txt on the USB stick. If
	 * the file exist then the details contained in it are used to populate the
	 * vLengths vector.
	 *
	 * @return An integer: 0 is success
	 */
	int loadProgram();

private:
	double pressureInBinary; ///< Measured pressure value at the ADC input
	bool exitThread;	 	 ///< Semaphore to terminate the thread

	CANSocket *can;		///< Pointer to CANSocket use by this DeviceControl
	exploringBB::GPIO 	*switch1GPIO,
						*switch2GPIO,
						*switch3GPIO; ///< Pointers to GPIO pins
};

extern DeviceControl *control;

#endif /* DEVICECONTROL_H_ */
