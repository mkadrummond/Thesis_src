/*
 * menuStructure.cpp
 *
 *  Created on: 18 Nov 2016
 *      Author: mkadrummond
 */

#include "menuStructure.h"
#include "menu.h"
#include "usb.h"
#include <cmath>
#include <iostream>
#include<fstream>
#include<string>
#include<sstream>


// Includes for shutting down
#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include "deviceControl.h"
#include "LCD.h"

int useRoot() {
	std::cout << "Entered useRoot()" << std::endl;

	// Clear the display and make sure the cursor is on
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(true);

	while(true) {
		deviceMenu->current = deviceMenu->map["root"];
		std::cout << "current is " << deviceMenu->current->getName() << std::endl;

		// Display the menu
		deviceMenu->displayMenu(deviceMenu->current);

		// Let the user move the cursor with the
		// encoder until they press the button
		while(deviceMenu->button->getValue() == exploringBB::LOW) {
			deviceMenu->updateDisplay(deviceMenu->current);
		}

		deviceMenu->current = deviceMenu->current->v[deviceMenu->current->vectorCursor];

		// Pass the user on to the useNode() function of
		// whichever menu node they selected
		if (deviceMenu->current->useNode() != 0) {
			break;
		}
	}

	return 0;
}


int useManual() {
	deviceMenu->current = deviceMenu->map["manual"];
	std::cout << "current is " << deviceMenu->current->getName() << std::endl;

	// Clear the display and
	// turn the cursor off
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	// Set the encoder position to 0
	deviceMenu->encoder->set_position(0);

	setPressure(&control->pressureSetPoint);

	control->pressureSetPoint = 0;

	deviceMenu->current = deviceMenu->map["root"];

	return 0;
}

int useAutomatic() {

	// Clear the display and
	// make sure the cursor is on
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(true);

	while(true) {
		deviceMenu->current = deviceMenu->map["automatic"];
		std::cout << "current is " << deviceMenu->current->getName() << std::endl;

		// Display the menu
		deviceMenu->displayMenu(deviceMenu->current);

		// Let the user move the cursor with the
		// encoder until they press the button
		while(deviceMenu->button->getValue() == exploringBB::LOW) {
			deviceMenu->updateDisplay(deviceMenu->current);
		}

		// If the user pressed the button while the cursor was at the top most item of the
		// menu (ie "back"), return them to the parent of the menu node. Otherwise pass them
		// on to the useNode() function of whichever menu node they selected
 		if (deviceMenu->current->vectorCursor == 0) {
 		//	deviceMenu->current = deviceMenu->map["root"];
 			return 0;
 		//	break;
		} else {
			deviceMenu->current->v[deviceMenu->current->vectorCursor]->useNode();
		}

	}

	return 0;
}

int useShutdown() {

	// Ask the user if they are sure before proceeding
	if (prompt("   Are you sure?")) {

		// Terminate the thread
		control->terminateThread();

		// Inform the user that the system is shutting down
		deviceMenu->lcd->clearDisplay();
		deviceMenu->lcd->displayCursor(false);
		deviceMenu->lcd->lcd_display_string("   Shutting down", 2);


		// Close the CAN port
		control->terminateCANbus();
//		control->can->close_port();
//		system("ifconfig can0 down");
		std::cout << "can down" << std::endl;

		// Stop the PRU timer
		control->terminatePRU();
		std::cout << "pru down" << std::endl;

		// Commit file system cache to disk
		sync();

		// Shutdown
		reboot(LINUX_REBOOT_CMD_POWER_OFF);

	} else {
		// If the user cancels the shutdown, return to the menu
		return 0;
	}

	return -1;
}



int useSetLengths() {

	// Clear the display and
	// make sure the cursor is active
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(true);

	while(true) {
		deviceMenu->current = deviceMenu->map["setLengths"];
		std::cout << "current is " << deviceMenu->current->getName() << std::endl;

		// Display the menu
		deviceMenu->displayMenu(deviceMenu->current);

		// Let the user move the cursor with the
		// encoder until they press the button
		while(deviceMenu->button->getValue() == exploringBB::LOW) {
			deviceMenu->updateDisplay(deviceMenu->current);
		}

		// If the user pressed the button while the cursor was at the
		// top most item of the menu (ie "back"), return them to the
		// parent of the menu node. Otherwise pass them on to the
		// useNode() function of whichever menu node they selected
 		if (deviceMenu->current->vectorCursor == 0) {
 			control->pressureSetPoint = 0;
			return 0;
 		} else {
 			deviceMenu->current->v[deviceMenu->current->vectorCursor]->useNode();
 		}
	}

	return 0;
}

int useLength() {
	// First note the pressure value already stored for this length
	float pressureSetP;
	pressureSetP = control->vLengths[deviceMenu->map["setLengths"]->vectorCursor - 1]->pressure;
	// Then, get input from the user on a new value
	// using the existing value as a starting point
	pressureSetP = setPressure(&pressureSetP);
	// Finally, update the stored value with the user defined value
	control->vLengths[deviceMenu->map["setLengths"]->vectorCursor - 1]->pressure = pressureSetP;

	// Do the same for the length
	int length = control->vLengths[deviceMenu->map["setLengths"]->vectorCursor - 1]->length;
	length = setLength(length);
	control->vLengths[deviceMenu->map["setLengths"]->vectorCursor - 1]->length = length;
	return 0;

}


int useAddNewLength() {
	std::cout << "Entered useAddNewLength()" << std::endl;

	float pressure = 0;
	int length;

	// If a length has been previously created, use its values to pre-set the new ones
	if (control->vLengths.size() > 0) {
		pressure = setPressure(&control->vLengths.back()->pressure);
		length = setLength(control->vLengths.back()->length);
	} else {

		pressure = setPressure(&pressure);
		length = setLength(0);
	}

	// Add another length object to vector of lengths
	// and pass it the attributes selected
	control->vLengths.push_back(new rig::Length);
	control->vLengths.back()->pressure = pressure;
	control->vLengths.back()->length = length;

	// Create a menu node to display on the LCD
	MenuNodeFactory NodeFactory;
	std::stringstream lengthName;
	lengthName << " Length " << deviceMenu->map["setLengths"]->v.size() - 1;
	deviceMenu->map["setLengths"]->v.insert( deviceMenu->map["setLengths"]->v.end() - 1, NodeFactory.createMenuNode(lengthName.str(), useLength));

    return 0;
}



int useStartProgram() {

	// Clear the display
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);


	// Ensure the CAN bus is initialised. Only necessary if the
	// valve was not plugged in at start-up
	control->initCANbus();

	// This function will mount the USB stick if available,
	// if not the user will be asked to insert one
	// or abort and return -1
	if (promptUSB() == -1) {
		return 0;
	}

//	AnalogIn *analogIn = new AnalogIn();

	// Ask which diameter of muscle is being used
	rig::muscleSize choiceOfMuscle = promptChooseMuscle();


	// Ask how many interpolation points should be used
	int interpolationPoints = promptInterpolatePressure();

	// Ask how many measurements should be recorded for each length
	int measurementCyclesPerLength = promptMeasurementCycles();

#if 0
	// Open a file stream to the USB stick
	ofstream datafile;
	datafile.open("/mnt/usb/data.txt", std::ios::out | std::ios::app);
	// Fill the first line of data.txt with all the pressure values
	// This makes reading the file later easier
	for (int k = 0; k < (int) control->vMeasurements.size(); k++) {
		datafile << "\t" <<  control->vMeasurements[k]->pressure;
	}
	datafile << "\n"; 	// End the line
#endif
	control->writePressurePointsToUSB();

	// This for-loop goes through the lengths that have been stored
	for (unsigned int i = 0; i < control->vLengths.size(); i++) {

		// Ask the user to release the muscle
		while (prompt("Unfasten the muscle") == 0) {
			if (prompt(" Abort measurement?")) {
				usb::unmountUSB();
				return 0;
			}
		}

		deviceMenu->lcd->clearDisplay();
		deviceMenu->lcd->displayCursor(false);
		deviceMenu->lcd->lcd_display_string(" Adjusting pressure", 2);
		deviceMenu->lcd->lcd_display_string("    Please wait", 3);
		// Bring the muscle to the pressure of this length
		control->pressureSetPoint = control->vLengths[i]->pressure;
		//adjustPressure(vLengths[i]->pressure);
		sleep(2);
		deviceMenu->lcd->clearDisplay();


		// Get the user to fix the muscle to this length
		while (prompt(" Fasten the muscle") == 0) {
			if (prompt(" Abort measurement?")) {
				usb::unmountUSB();
				return 0;
			}
		}

		// Get the user to fix the muscle to this length
		while (prompt(" Start measurement?") == 0) {
			if (prompt(" Abort measurement?")) {
				usb::unmountUSB();
				return 0;
			}
		}

		// Transfer this variable to another with which we can count down with
		int count = measurementCyclesPerLength;

		while (count > 0) {
			deviceMenu->lcd->clearDisplay();
			deviceMenu->lcd->displayCursor(false);

			// This while loop goes through the pressure points measuring the force at each
			unsigned int j = i + i*interpolationPoints;
			while (j < control->vMeasurements.size()) {

				deviceMenu->lcd->clearDisplay();

				// Display the static elements on the LCD
				deviceMenu->lcd->lcd_display_string("Pressure (bar)", 1);
				deviceMenu->lcd->lcd_display_string(" Set point", 2);
				deviceMenu->lcd->lcd_display_float(control->vMeasurements[j]->pressure, 6); // pressure set point
				deviceMenu->lcd->lcd_display_string(" Measured", 3);
				deviceMenu->lcd->lcd_display_string("Force (N)", 4);

				// Start the control function in a new thread and
				// pass it the next pressure set point to aim for
				// from the vMeasurements vector
				control->pressureSetPoint = control->vMeasurements[j]->pressure;

				// The controller will try to get within a 0.01bar the set point.
				// To determine if this margin is reached, the error between the measured
				// pressure and the set point is calculated
				float margin = 0.01;
				float error;
				int counter = 120;

				float force;	// measured force

				// The valve is prone to big overshoot when a new pressure value is set
				// The use of the addition count variable acts as a timer, and only
				// decrements when the error is within the margins
				do {
					// Get the RMS of the difference between set point and
					// measured pressure. The RMS ensures it is positive
					error = sqrt(pow(control->vMeasurements[j]->pressure - control->pressureMeasured/1000, 2));

					if (error < margin) {
						counter--;
					}

					force = control->measureForce(choiceOfMuscle);

					// Negative measured pressure causes problems with the display
					if(control->pressureMeasured < 0) {
						control->pressureMeasured = 0;
					}

					// Update the display with the latest
					// measured pressure and force values
					deviceMenu->lcd->lcd_display_float(control->pressureMeasured/1000, 7);
					deviceMenu->lcd->lcd_display_int(static_cast<int>(force), 8);

				} while (error > margin || counter != 0); // Continue until the error is within bounds

				// Wait another second to make sure we really are at the set point
				sleep(1);

				// Store the measured force in the vMeasurements vector
				control->vMeasurements[j]->force = force;

				// Cycle to next pressure point
				j++;

			}	// End of pressure measurements loop

			deviceMenu->lcd->clearDisplay();
			deviceMenu->lcd->displayCursor(false);

			deviceMenu->lcd->lcd_display_string(" Writing to USB...", 2);
			control->writeMeasurementsToUSB(i);
			sleep(1);

			// Repeat the measurement if the user chose this
			count--;
			if (count > 0) {
				deviceMenu->lcd->clearDisplay();
				deviceMenu->lcd->lcd_display_string("     Repeating", 1);
				deviceMenu->lcd->lcd_display_string("    measurement", 2);
				deviceMenu->lcd->lcd_display_string("Measurements left:", 4);
				deviceMenu->lcd->lcd_display_int(count, 8);
				sleep(2);
			}

		}

		// The vMeasurements vector is "cleared" for each new length measurement
		// Initialise all force values in the vMeasurements vector to 0.
		// This ensures that pressure points not measured at subsequent lengths
		// have a corresponding force measurement of 0
		for (int k = 0; k < (int) control->vMeasurements.size(); k++) {
			control->vMeasurements[k]->force = 0;
		}


		deviceMenu->lcd->clearDisplay();
		deviceMenu->lcd->lcd_display_string(" Adjusting pressure", 2);
		deviceMenu->lcd->lcd_display_string("    Please wait", 3);
		// Bring the muscle to the pressure of this length
		control->pressureSetPoint = control->vLengths[i]->pressure;
		sleep(2);
		deviceMenu->lcd->clearDisplay();


	} // End of length loop

	// Ask the user to release the muscle
	while (prompt("Unfasten the muscle") == 0) {
		if (prompt(" Abort measurement?")) {
//			usb::unmountUSB();
			return 0;
		}
	}

	// Return the valve and muscle to the zero position
	control->pressureSetPoint = 0;

	deviceMenu->lcd->lcd_display_string("Measurement success", 2);

	// Unmount the USB stick
//	usb::unmountUSB();

	return 0;
}


int useSave() {

	while (prompt(" Save the program?") == 0) {
			return 0;
	}

	// This function will mount the USB stick if available,
	// if not the user will be asked to insert one
	// or abort and return -1
	if (promptUSB() == -1) {
		return -1;
	}

	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	if (control->saveProgram()) {
		perror("control->saveProgram()");
		deviceMenu->lcd->lcd_display_string("Error opening file", 2);
		sleep(2);
	} else {
		std::cout << "Program saved to USB" << std::endl;
		deviceMenu->lcd->lcd_display_string("  Writing to USB", 2);
		sleep(2);
	}

	// Unmount the USB stick
	usb::unmountUSB();

	return 0;
}

int useLoad() {

	while (prompt(" Load the program?") == 0) {
			return 0;
	}

	// Clear the display
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	// This will mount the USB stick if available,
	// if not the user will be asked to insert one
	// or abort and return -1
	if (promptUSB() == -1) {
		return -1;
	}

	deviceMenu->lcd->clearDisplay();

	if (control->loadProgram()) {
		perror("control->loadProgram()");
		deviceMenu->lcd->lcd_display_string("Error opening file", 2);
		sleep(2);
	} else {
		std::cout << "Program loaded" << std::endl;
		deviceMenu->lcd->lcd_display_string("   Program loaded", 2);
		sleep(2);
	}

	// Delete the appropriate MenuNodes
	deviceMenu->map["setLengths"]->v.erase(deviceMenu->map["setLengths"]->v.begin()+1,
													deviceMenu->map["setLengths"]->v.end()-1);

	// Create a corresponding MenuNodes to display
	MenuNodeFactory NodeFactory;

	for(int i = 0; i < static_cast<int>(control->vLengths.size()); i++ ) {
		std::stringstream lengthName;
		lengthName << " Length " << i + 1;
		deviceMenu->map["setLengths"]->v.insert(deviceMenu->map["setLengths"]->v.end() - 1,
													NodeFactory.createMenuNode(lengthName.str(), useLength));
	}

	// Unmount the USB stick
	usb::unmountUSB();

	return 0;
}


// A function to display a binary question. The question
// is passed to the function as a string attribute and
// the user can select OK or Cancel
int prompt(string str) {

	// Clear the display. Cursor is on
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(true);

	// Display the binary question on the first line
	deviceMenu->lcd->lcd_display_string(str, 1);

	// Display the options to the user
	deviceMenu->lcd->lcd_display_string("         OK", 3);
	deviceMenu->lcd->lcd_display_string("       Cancel", 4);

	// Start the cursor at OK
	deviceMenu->encoder->set_position(4);

	// Variable and while loop to keep track of the user's
	// encoder movement and selection
	int cursor;
	while(deviceMenu->button->getValue() == exploringBB::LOW) {

		if (deviceMenu->encoder->get_position() > 0) {
			cursor = 1;
			deviceMenu->encoder->set_position(1);
		} else {
			cursor = 0;
			deviceMenu->encoder->set_position(0);
		}

		if (cursor) {
			deviceMenu->lcd->lcd_write(0x9C, 0);
		} else {
			deviceMenu->lcd->lcd_write(0xDA, 0);
		}
	}

	return cursor;
}



rig::muscleSize promptChooseMuscle() {

	// Clear the display
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	deviceMenu->lcd->lcd_display_string("  Which muscle is", 1);
	deviceMenu->lcd->lcd_display_string("    being used", 2);

	int selector;
	rig::muscleSize muscleChoice;
	deviceMenu->encoder->set_position(0);

	while(deviceMenu->button->getValue() == exploringBB::LOW) {

		selector = deviceMenu->encoder->get_position()/4;

		if (selector < 1) {
			selector = 0;
			deviceMenu->encoder->set_position(0);
			deviceMenu->lcd->lcd_display_string("                 5mm", 4);
			muscleChoice = rig::muscle5mm;
		}
		if (selector == 1) {
			deviceMenu->lcd->lcd_display_string("                10mm", 4);
			muscleChoice = rig::muscle10mm;

		}
		if (selector == 2) {
			deviceMenu->lcd->lcd_display_string("                20mm", 4);
			muscleChoice = rig::muscle20mm;

		}
		if (selector > 2) {
			selector = 3;
			deviceMenu->encoder->set_position(12);
			deviceMenu->lcd->lcd_display_string("                40mm", 4);
			muscleChoice = rig::muscle40mm;

		}

	}

	switch (muscleChoice) {
	case rig::muscle5mm:
		control->analogSwitch(0);
		break;
	case rig::muscle10mm:
		control->analogSwitch(1);
		break;
	case rig::muscle20mm:
		control->analogSwitch(2);
		break;
	case rig::muscle40mm:
		control->analogSwitch(3);
	}

	// Clear the display
	deviceMenu->lcd->clearDisplay();
	return muscleChoice;
}

int promptInterpolatePressure() {

	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	deviceMenu->lcd->lcd_display_string("Select the number of", 1);
	deviceMenu->lcd->lcd_display_string("interpolation points", 2);

	int interpolationPoints;
	deviceMenu->encoder->set_position(0);

	while(deviceMenu->button->getValue() == exploringBB::LOW) {

		interpolationPoints = deviceMenu->encoder->get_position()/4;

		if (interpolationPoints < 0) {
			interpolationPoints = 0;
			deviceMenu->encoder->set_position(0);
		}
		if (interpolationPoints > 3) {
			interpolationPoints = 3;
			deviceMenu->encoder->set_position(12);
		}

		deviceMenu->lcd->lcd_display_int(interpolationPoints, 8);
	}

	control->vMeasurements.clear();

	for(unsigned int i = 0; i < control->vLengths.size(); i++) {
		if (i == 0) {
			control->vMeasurements.push_back(new rig::Measurement);
			control->vMeasurements.back()->pressure = control->vLengths[i]->pressure;
			control->vMeasurements.back()->force = 0;
		} else {
			float spacer = (control->vLengths[i]->pressure - control->vLengths[i - 1]->pressure) / (interpolationPoints + 1);
			for (int j = 0; j <= interpolationPoints; j++) {
				control->vMeasurements.push_back(new rig::Measurement);
				control->vMeasurements.back()->pressure = control->vLengths[i - 1]->pressure + spacer*(j + 1);
				control->vMeasurements.back()->force = 0;
			}
		}
	}

	// Clear the display
	deviceMenu->lcd->clearDisplay();

	return interpolationPoints;
}

int promptMeasurementCycles() {

	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	deviceMenu->lcd->lcd_display_string(" Select measurement", 1);
	deviceMenu->lcd->lcd_display_string(" cycles per length", 2);

	int measurementCycles;
	deviceMenu->encoder->set_position(4);

	while(deviceMenu->button->getValue() == exploringBB::LOW) {

		measurementCycles = deviceMenu->encoder->get_position()/4;

		if (measurementCycles < 1) {
			measurementCycles = 1;
			deviceMenu->encoder->set_position(4);
		}

		deviceMenu->lcd->lcd_display_int(measurementCycles, 8);
	}

	// Clear the display
	deviceMenu->lcd->clearDisplay();

	return measurementCycles;
}


// This function tries to mount the USB stick. If this
// fails, the user will be asked to insert a stick.
int promptUSB() {

	// Try to mount the USB stick
	while (usb::mountUSB() < 0 ){
		// Warn user that no stick was found
		deviceMenu->lcd->clearDisplay();
		deviceMenu->lcd->lcd_display_string("   No USB stick!", 2);
		sleep(2); 	// Wait, so that the use sees the message
		// Call on the user to insert a stick
		while (prompt("  Insert USB stick") == 0) {
			// User has selected Cancel. Ask if they are sure
			if (prompt("  Abort USB mount?")) {
				// Return. USB stick has not been mounted
				return -1;
			}
		}

	}

	// Return, USB stick mounted successfully
	return 0;
}

#if 0

int Menu::writeData2File(std::ofstream datafile, int i) {

	if (datafile.is_open()) {
		std::cout << "File successfully opened" << std::endl;
		//lcd_display_string(" Writing to USB...", 2);
		datafile <<  control->vLengths[i]->length << ":\t";
		for (int k = 0; k < (int) control->vMeasurements.size(); k++) {
			datafile <<  static_cast<int>(control->vMeasurements[k]->force) << "\t";
		}
		datafile << "\n";
		datafile.close();
	} else {
		std::cout << "Error opening file!" << std::endl;
		//lcd_display_string("Error opening file!", 2);
	}

	//sleep(2);

	return 0;
}
#endif

#if 0
float setPressure() {

	float pressureSetPoint = static_cast<float>(deviceMenu->encoder->get_position())/80;

	// The set point can't be negative
	if(pressureSetPoint < 0) {
		pressureSetPoint = 0;
		deviceMenu->encoder->set_position(0);
	}

	control->pressureSetPoint = pressureSetPoint;

	// Display the set point and the measured pressures
	lcd_display_float(pressureSetPoint, 6);

	// Negative measured pressures cause problems with the display
	if (control->pressureMeasured < 0) {
		lcd_display_float(0, 7);
	} else {
		lcd_display_float(static_cast<float>(control->pressureMeasured)/1000, 7);
	}

	return control->pressureSetPoint;
}
#endif

float setPressure(float *pressure) {

	std::cout << "Entered setPressure()" << std::endl;

	// Clear display and turn the cursor off
	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	// Set the encoder position to the value passed to setPressure()
	deviceMenu->encoder->set_position(static_cast<int>(*pressure*80));

	// Put the static elements on the display
	deviceMenu->lcd->lcd_display_string("Pressure (bar)", 1);
	deviceMenu->lcd->lcd_display_string(" Set point", 2);
	deviceMenu->lcd->lcd_display_string(" Measured", 3);

	while(deviceMenu->button->getValue() == exploringBB::LOW) {

		control->pressureSetPoint = static_cast<float>(deviceMenu->encoder->get_position())/80;

		// The set point can't be negative
		if(control->pressureSetPoint < 0) {
			control->pressureSetPoint = 0;
			deviceMenu->encoder->set_position(0);
		}

		// Display the set point and the measured pressures
		deviceMenu->lcd->lcd_display_float(control->pressureSetPoint, 6);

		// Negative measured pressures cause problems with the display
		if (control->pressureMeasured < 0) {
			deviceMenu->lcd->lcd_display_float(0, 7);
		} else {
			deviceMenu->lcd->lcd_display_float(static_cast<float>(control->pressureMeasured)/1000, 7);
		}
	}

	// Return encoder to its zero position
	deviceMenu->encoder->set_position(0);

	return control->pressureSetPoint;
}


int setLength(int length) {

	deviceMenu->lcd->clearDisplay();
	deviceMenu->lcd->displayCursor(false);

	deviceMenu->encoder->set_position(static_cast<int>(length*4));

	deviceMenu->lcd->lcd_display_string("Length (mm)", 2);

	usleep(100000);


	while(deviceMenu->button->getValue() == exploringBB::LOW) {

		length = deviceMenu->encoder->get_position()/4;

		// Don't go negative
		if(length < 0) {
			length = 0;
		}

		deviceMenu->lcd->lcd_display_int(length, 6);
	}

	deviceMenu->encoder->set_position(0);

	return length;
}
