//============================================================================
// Name        : Beaglebone.cpp
// Author      : Michael Drummond
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

//A simple SocketCAN example
#include "CAN.h"

// includes for the gpio buttons
#include "GPIO.h"
#include <unistd.h>
#include <pthread.h>



//display
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// Include for the display
#include "lcdDisplay.h"

// Include for the menu
#include "menu.h"

// eQEP software taken from
// https://github.com/Teknoman117/beaglebot/tree/master/encoders/api/c%2B%2B
// Includes for the rotary encoder
#include "eqep.h"

//PRU
#include <stdlib.h>
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#define PRU_NUM 0   	// using PRU0 for the timer
#define PRU_BIN_ADDR 	"/home/mkadrummond/pru/timer.bin"	//

// PID controller
// from: https://gist.github.com/bradley219/5373998
#include "pid.h"

// Threads
#include "thread.h"

//#include"adc.h"
// Includes for the ADC input from the force sensor
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
// Linux path to read the ADC input from the force sensor
#define LDR_PATH "/sys/bus/iio/devices/iio:device0/in_voltage"

// Mounting USB
#include <sys/mount.h>
#include "usb.h"

// Data
#include "data.h"

// For calculating the error to the pressure set point
#include <cmath>

// Includes for shutting down
#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include <stdio.h>
#include <iostream>
#include <sstream>
using namespace std;
using namespace exploringBB; // for the GPIO buttons

#define MAX10V	1
#define MAX3V3 	2
#define MAX6V	3

enum muscleSize { muscle5mm, muscle10mm, muscle20mm, muscle40mm};

/*** GLOBAL VARIABLES ***/
float pressureSP;
double pressureInBinary;
double pressureInBar;

std::vector<data::Length*> vLengths;
std::vector<data::Measurement*> vMeasurements;

bool exitThread;


/*** GLOBAL OBJECT POINTERS ***/

// General purpose IO
GPIO 	*button1GPIO,
		*ledGPIO,
		*switch1GPIO,
		*switch2GPIO,
		*switch3GPIO;

// Rotary encoder
eQEP *encoder;

// CAN-bus
CANSocket *can0;

//controlThread *controlThr;

// Menu structure
Menu::MenuNode *root;
	Menu::MenuNode *manual;
	Menu::MenuNode *automatic;
		Menu::MenuNode *setLengths;
			Menu::MenuNode *addNewLength;
		Menu::MenuNode *startProgram;
		Menu::MenuNode *save;
		Menu::MenuNode *load;
	Menu::MenuNode *shutdownBBB;


/*** FUNCTION DECLARATIONS ***/

// Functions to display and navigate the menu
int displayMenu(Menu::MenuNode*);
int updateDisplay(Menu::MenuNode*);

// Functions for the Menu Nodes. These are linked to
// the useNode() functions of their respective menuNodes
int useRoot();
int useManual();
int useShutdown();
int useSetLengths();
int useAddNewLength();
int useAutomatic();
int useStartProgram();
int useSave();
int useLoad();

// Additional functions related to the menu that
// prompt the user for further information or actions
int prompt(string);
muscleSize promptChooseMuscle();
int promptInterpolatePressure();
int promptMeasurementCycles();
int promptUSB();

// Backend functions
float setPressure(float*);
int setLength(int);
int adjustPressure(float);
void *controlfunc(void*);
int readAnalog(int);
float measureForce(muscleSize);
float binary2Bar(int);
int initCANbus();



int main() {

	if(getuid()!=0){
		cout << "You must run this program as root. Exiting." << endl;
		cout << getuid() << endl;
		return -1;
	}

	/*** Initialise the LCD  ***/
	initI2C();
	initLCD();

	/*** Prepare the rotary encoder, buttons and LEDs ***/

    // Create quadrature encoder
    encoder = new eQEP(eQEP2, eQEP::eQEP_Mode_Absolute);
    encoder->set_period(10000000L); // Encoder polled every 10ms

    // Create GPIO objects
    // Button:
    button1GPIO = new GPIO(88);        	//button down
    button1GPIO->setDirection(INPUT);   // button 1 is an input
    button1GPIO->setEdgeType(RISING);   //wait for rising edge

    // LED:
    ledGPIO = new GPIO(67);        		//LED
    ledGPIO->setDirection(OUTPUT); 		//LED is an output
    ledGPIO->streamOpen();         		//fast write to LED
    ledGPIO->streamWrite(LOW);     		//turn the LED off

    // Analogue switch for reading the force:
    switch1GPIO = new GPIO(112); 	// Connects AIN1
    switch2GPIO = new GPIO(111);	// Connects AIN2
    switch3GPIO = new GPIO(110);	// Connects AIN3

    switch1GPIO->setDirection(OUTPUT);
    switch2GPIO->setDirection(OUTPUT);
    switch3GPIO->setDirection(OUTPUT);

    switch1GPIO->streamOpen();
    switch2GPIO->streamOpen();
    switch3GPIO->streamOpen();

	//exitThread = false;

	/*** Prepare the GUI menu structure ***/
	// Create the menu node elements
	root = new Menu::MenuNode("root", useRoot);
		manual = new Menu::MenuNode(" Manual", root, useManual);
		automatic = new Menu::MenuNode(" Automatic", root, useAutomatic);
			setLengths = new Menu::MenuNode(" Set lengths", automatic, useSetLengths);
				addNewLength = new Menu::MenuNode(" Add new length", setLengths, useAddNewLength);
			startProgram = new Menu::MenuNode(" Start program", automatic, useStartProgram);
			save = new Menu::MenuNode(" Save program", automatic, useSave);
			load = new Menu::MenuNode(" Load program", automatic, useLoad);
		shutdownBBB = new Menu::MenuNode(" Shutdown", root, useShutdown);

	cout << automatic->v[0]->getName() << endl;
	cout << automatic->v[1]->getName() << endl;
	cout << automatic->v[2]->getName() << endl;
	cout << automatic->v[3]->getName() << endl;

	/*** Initialise the PRU on the BBB  ***/

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


	/*** Initialise the CAN bus and Valve ***/
	initCANbus();

	can0 = new CANSocket("can0");

	sleep(1);

	// Initialise the valve
	int ret = can0->initNode(0x01);
	if (ret < 0) {
		perror("CANbus");
	}

	// Start the thread that runs the control loop
	startThread(controlfunc, &pressureSP);

	clearDisplay();

	// On start, the menu will be at the root node
	root->useNode();

return 0;
}


/*** FUNCTIONS ***/

int useRoot() {

	// Clear the display and make sure the cursor is on
	clearDisplay();
	displayCursor(true);

	while(true) {

		// Display the menu
		displayMenu(root);

		// Let the user move the cursor with the
		// encoder until they press the button
		while(button1GPIO->getValue() == LOW) {
			updateDisplay(root);
		}

		// Pass the user on to the useNode() function of
		// whichever menu node they selected
		if (root->v[root->vectorCursor]->useNode() != 0) {
			break;
		}
	}

	return 0;
}

int useManual() {

	// Clear the display and
	// turn the cursor off
	clearDisplay();
	displayCursor(false);

	// Set the encoder position to 0
	encoder->set_position(0);

	// This function starts a thread with control function to
	// set the muscle internal pressure with the encoder
	// setPressure(0);
	setPressure(&pressureSP);

	// Bring the valve and muscle back to the zero position
	//adjustPressure(0);



	return 0;
}

int useAutomatic() {

	// Clear the display and
	// make sure the cursor is on
	clearDisplay();
	displayCursor(true);

	while(true) {

		// Display the menu
		displayMenu(automatic);

		// Let the user move the cursor with the
		// encoder until they press the button
		while(button1GPIO->getValue() == LOW) {
			updateDisplay(automatic);
		}

		// If the user pressed the button while the cursor was at the top most item of the
		// menu (ie "back"), return them to the parent of the menu node. Otherwise pass them
		// on to the useNode() function of whichever menu node they selected
 		if (automatic->vectorCursor == 0) {
 			return 0;
		} else {
			automatic->v[automatic->vectorCursor]->useNode();
		}

	}

	return 0;
}

int useShutdown() {

	// Ask the user if they are sure before proceeding
	if (prompt("   Are you sure?")) {

		// Terminate the thread
		exitThread = true;

		// Inform the user that the system is shutting down
		clearDisplay();
		displayCursor(false);
		lcd_display_string("   Shutting down", 2);
		cout << "display" << endl;

		// Close the CAN port
		can0->close_port();
		system("ifconfig can0 down");
		cout << "can down" << endl;

		// Stop the PRU timer
		prussdrv_exit ();
		cout << "pru down" << endl;

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
	clearDisplay();
	displayCursor(true);

	while(true) {

		// Display the menu
		displayMenu(setLengths);

		// Let the user move the cursor with the
		// encoder until they press the button
		while(button1GPIO->getValue() == LOW) {
			updateDisplay(setLengths);
		}

		// If the user pressed the button while the cursor was at the
		// top most item of the menu (ie "back"), return them to the
		// parent of the menu node. Otherwise pass them on to the
		// useNode() function of whichever menu node they selected
 		if (setLengths->vectorCursor == 0) {
 			pressureSP = 0;
			return 0;
 		} else {
 			setLengths->v[setLengths->vectorCursor]->useNode();
 		}
	}

	return 0;
}

int useLength() {

	// First note the pressure value already stored for this length
	float pressureSetP;
	pressureSetP = vLengths[setLengths->vectorCursor - 1]->pressure;
	// Then, get input from the user on a new value
	// using the existing value as a starting point
	pressureSetP = setPressure(&pressureSetP);
	// Finally, update the stored value with the user defined value
	vLengths[setLengths->vectorCursor - 1]->pressure = pressureSetP;

	// Do the same for the length
	int length = vLengths[setLengths->vectorCursor - 1]->length;
	length = setLength(length);
	vLengths[setLengths->vectorCursor - 1]->length = length;

	return 0;
}

int useAddNewLength() {

	float pressure = 0;
	int length;

	// Get input from the user as to the pressure set point and
	// length of the muscle. If a length has been previously
	// created, use its values to pre-set the new ones
	if (vLengths.size() > 0) {
		pressure = setPressure(&vLengths.back()->pressure);
		length = setLength(vLengths.back()->length);
	} else {
		pressure = setPressure(&pressure);
		length = setLength(0);
	}

	// Add another length object to vector of lengths
	// and pass it the attributes selected
	vLengths.push_back(new data::Length);
	vLengths.back()->pressure = pressure;
	vLengths.back()->length = length;

	// Create a menu node to display on the LCD
	Menu::MenuNodeFactory NodeFactory;
	stringstream lengthName;
	lengthName << " Length " << setLengths->v.size() - 1;
	setLengths->v.insert( setLengths->v.end() - 1, NodeFactory.createMenuNode(lengthName.str(), useLength));

    return 0;
}

float setPressure(float *pressure) {

	// Clear display and turn the cursor off
	clearDisplay();
	displayCursor(false);

	// Set the encoder position to the value passed to setPressure()
	encoder->set_position(static_cast<int>(*pressure*80));

	// Put the static elements on the display
	lcd_display_string("Pressure (bar)", 1);
	lcd_display_string(" Set point", 2);
	lcd_display_string(" Measured", 3);

	while(button1GPIO->getValue() == LOW) {

		pressureSP = static_cast<float>(encoder->get_position())/80;

		// The set point can't be negative
		if(pressureSP < 0) {
			pressureSP = 0;
			encoder->set_position(0);
		}

		// Display the set point and the measured pressures
		lcd_display_float(pressureSP, 6);

		// Negative measured pressures cause problems with the display
		if (pressureInBar < 0) {
			lcd_display_float(0, 7);
		} else {
			lcd_display_float(static_cast<float>(pressureInBar)/1000, 7);
		}
	}

	// Return encoder to its zero position
	encoder->set_position(0);

	return pressureSP;
}

int setLength(int length) {

	usleep(10000);

	encoder->set_position(static_cast<int>(length*4));

	clearDisplay();
	displayCursor(false);

	lcd_display_string("Length (mm)", 2);

	while(button1GPIO->getValue() == LOW) {

		length = encoder->get_position()/4;

		// Don't go negative
		if(length < 0) {
			length = 0;
		}

		lcd_display_int(length, 6);
	}

	encoder->set_position(0);

	return length;
}

int useStartProgram() {

	// Clear the display
	clearDisplay();
	displayCursor(false);

	// Ensure the CAN bus is initialised. Only necessary if the
	// valve was not plugged in at start-up
	initCANbus();

	// This function will mount the USB stick if available,
	// if not the user will be asked to insert one
	// or abort and return -1
	if (promptUSB() == -1) {
		return 0;
	}

	// Ask which diameter of muscle is being used
	muscleSize choiceOfMuscle = promptChooseMuscle();

	// Ask how many interpolation points should be used
	int interpolationPoints = promptInterpolatePressure();

	// Ask how many measurements should be recorded for each length
	int measurementCyclesPerLength = promptMeasurementCycles();

	// Open a file stream to the USB stick
	ofstream datafile;
	datafile.open("/mnt/usb/data.txt", ios::out | ios::app);
	// Fill the first line of data.txt with all the pressure values
	// This makes reading the file later easier
	for (int k = 0; k < (int) vMeasurements.size(); k++) {
		datafile << "\t" <<  vMeasurements[k]->pressure;
	}
	datafile << "\n"; 	// End the line

	// This for-loop goes through the lengths that have been stored
	for (unsigned int i = 0; i < vLengths.size(); i++) {

		// Ask the user to release the muscle
		while (prompt("Unfasten the muscle") == 0) {
			if (prompt(" Abort measurement?")) {
				usb::unmountUSB();
				return 0;
			}
		}

		clearDisplay();
		lcd_display_string(" Adjusting pressure", 2);
		lcd_display_string("    Please wait", 3);
		// Bring the muscle to the pressure of this length
		pressureSP = vLengths[i]->pressure;
		//adjustPressure(vLengths[i]->pressure);
		sleep(2);
		clearDisplay();


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
			clearDisplay();
			displayCursor(false);

			// This while loop goes through the pressure points measuring the force at each
			unsigned int j = i + i*interpolationPoints;
			while (j < vMeasurements.size()) {

				clearDisplay();

				// Display the static elements on the LCD
				lcd_display_string("Pressure (bar)", 1);
				lcd_display_string(" Set point", 2);
				lcd_display_float(vMeasurements[j]->pressure, 6); // pressure set point
				lcd_display_string(" Measured", 3);
				lcd_display_string("Force (N)", 4);

				// Start the control function in a new thread and
				// pass it the next pressure set point to aim for
				// from the vMeasurements vector
				pressureSP = vMeasurements[j]->pressure;
				//startThread(controlfunc, &vMeasurements[j]->pressure);

				// A little pause prevents an initial overshoot from catching
				// the pressure before it reaches its set point
				sleep(1);

				// The controller will try to get within a 0.1% margin of the set point.
				// To determine if this margin is reached, the error between the measured
				// pressure and the set point is calculated
				float margin = 0.01;
				float error;
				int count = 120;

				float force;	// measured force

				// The valve is prone to big overshoot when a new pressure value is set
				// The use of the addition count variable acts as a timer, and only
				// decrements when the error is within the margins
				do {
					// Get the RMS of the difference between set point and
					// measured pressure. The RMS ensures it is positive
					error = sqrt(pow(vMeasurements[j]->pressure - pressureInBar/1000, 2));

					if (error < margin) {
						count--;
					}

					force = measureForce(choiceOfMuscle);

					// Negative measured pressure causes problems with the display
					if(pressureInBar < 0) {
						pressureInBar = 0;
					}

					// Update the display with the latest
					// measured pressure and force values
					lcd_display_float(pressureInBar/1000, 7);
					lcd_display_int(static_cast<int>(force), 8);

				} while (error > margin || count != 0); // Continue until the error is within bounds

				// Wait another second to make sure we really are at the set point
				sleep(1);

				// Store the measured force in the vMeasurements vector
				vMeasurements[j]->force = force;

				// Cycle to next pressure point
				j++;

			}	// End of pressure measurements loop

			clearDisplay();
			displayCursor(false);

			if (datafile.is_open()) {
				cout << "File successfully opened" << endl;
				lcd_display_string(" Writing to USB...", 2);
				datafile <<  vLengths[i]->length << ":\t";
				for (int k = 0; k < (int) vMeasurements.size(); k++) {
					datafile <<  static_cast<int>(vMeasurements[k]->force) << "\t";
				}
				datafile << "\n";

			} else {
				cout << "Error opening file!" << endl;
				lcd_display_string("Error opening file!", 2);
			}

			sleep(1);

			// Repeat the measurement if the user chose this
			count--;
			if (count > 0) {
				clearDisplay();
				lcd_display_string("     Repeating", 1);
				lcd_display_string("    measurement", 2);
				lcd_display_string("Measurements left:", 4);
				lcd_display_int(count, 8);
				sleep(2);
			}

		}

		// The vMeasurements vector is "cleared" for each new length measurement
		// Initialise all force values in the vMeasurements vector to 0.
		// This ensures that pressure points not measured at subsequent lengths
		// have a corresponding force measurement of 0
		for (int k = 0; k < (int) vMeasurements.size(); k++) {
			vMeasurements[k]->force = 0;
		}


		clearDisplay();
		lcd_display_string(" Adjusting pressure", 2);
		lcd_display_string("    Please wait", 3);
		// Bring the muscle to the pressure of this length
		pressureSP = vLengths[i]->pressure;
		//adjustPressure(vLengths[i]->pressure);
		sleep(2);
		clearDisplay();


	} // End of length loop

	// Ask the user to release the muscle
	while (prompt("Unfasten the muscle") == 0) {
		if (prompt(" Abort measurement?")) {
			usb::unmountUSB();
			return 0;
		}
	}

	// Return the valve and muscle to the zero position
	pressureSP = 0;
	//adjustPressure(0);

	datafile.close();

	lcd_display_string("Measurement success", 2);

	// Unmount the USB stick
	usb::unmountUSB();

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

	clearDisplay();
	displayCursor(false);

	ofstream programFile;
	programFile.open("/mnt/usb/program.txt", ios::out | ios::trunc);
	if (programFile.is_open()) {
		cout << "File successfully opened" << endl;
		lcd_display_string("  Writing to USB", 2);

		// Total number of Lengths; helpful for when we want to load the program later:
		programFile << vLengths.size() << "\n";

		for(unsigned int i = 0; i < vLengths.size(); i++) {
			programFile << vLengths[i]->length << "\t";
			programFile << vLengths[i]->pressure << "\t\n";

		}

		programFile.close();

	} else {
		perror("program.txt");
		cout << "Error opening file" << endl;
		lcd_display_string("Error opening file!", 2);
	}

	// Pause, so that the user sees the message
	sleep(1);

	// Unmount the USB stick
	usb::unmountUSB();

	return 0;
}

int useLoad() {

	while (prompt(" Load the program?") == 0) {
			return 0;
	}

	// Clear the display
	clearDisplay();
	displayCursor(false);

	// This function will mount the USB stick if available,
	// if not the user will be asked to insert one
	// or abort and return -1
	if (promptUSB() == -1) {
		return -1;
	}

	clearDisplay();

	ifstream programFile("/mnt/usb/program.txt", ios::in);

	// Delete any previously stored lengths
	vLengths.clear();

	Menu::MenuNodeFactory NodeFactory;

	int n;
	float pressure;
	int length;

	if (programFile >> n) {

		// Delete any previously stored lengths
		vLengths.clear();
		setLengths->v.erase(setLengths->v.begin()+1, setLengths->v.end()-1);

		while (n > 0) {

			if (programFile >> length >> pressure) {
				vLengths.push_back(new data::Length);
				vLengths.back()->pressure = pressure;
				vLengths.back()->length = length;

				// Create a corresponding menu node to display on the display
				stringstream lengthName;
				lengthName << " Length " << setLengths->v.size() - 1;
				setLengths->v.insert( setLengths->v.end() - 1, NodeFactory.createMenuNode(lengthName.str(), useLength));

			} else {
				perror("program.txt");
				lcd_display_string("Error loading file", 2);
				return EXIT_FAILURE;
			}
			n--;
		}

		programFile.close();
		lcd_display_string("   Program loaded", 2);
		sleep(2);

	} else {
		perror("program.txt");
		lcd_display_string("Error opening file!", 2);
		sleep(2);
	}

	// Unmount the USB stick
	usb::unmountUSB();

	return 0;
}


void *controlfunc(void *args) {

	float pressureOutput;
	float dt = 0.01;
    double max = 1;
    double min = -1;
    float Kp = 0.7;
    float Ki = 1;
    float Kd = 0;

    PID pidControl(dt, max, min, Kp, Kd, Ki);

	float *pres = static_cast<float*>(args);

	exitThread = false;

	while (exitThread == false) {

		// Wait for event completion from PRU
		prussdrv_pru_wait_event (PRU_EVTOUT_0);

		prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

		// read the adc
		pressureInBinary = readAnalog(0);

 		pressureInBar = binary2Bar(pressureInBinary);

 		pressureOutput = pidControl.calculate( (*pres)*1000, pressureInBar );

		can0->sendFrame(0x100, pressureOutput);

	}

	cout << "Thread closed" << endl;
	exitThread = false;

	return 0;
}


float binary2Bar(int adcValue) {
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


// function for the force sensor ADC input
int readAnalog(int number){
	stringstream ss;
    ss << LDR_PATH << number << "_raw";
    fstream fs;
    fs.open(ss.str().c_str(), fstream::in);
    fs >> number;
    fs.close();
    return number;
}


int displayMenu(Menu::MenuNode* current) {

	int *displayOffset = &current->displayOffset;
	int *vc = &current->vectorCursor;

	// This makes sure that the cursor is back at the same
	// position in the menu when one returns to a menuNode
	encoder->set_position((*vc)*-4);

	// Clear the display
	clearDisplay();

	// turn the cursor off because it's distracting
	displayCursor(false);

	for(int i = 0; i < (int)current->v.size() && i<4; i++) {
		if(*displayOffset+i == 0 && current !=  root) {
			lcd_display_string(" Back", 1);
		} else {
			lcd_display_string(current->v[*displayOffset + i]->getName(), i+1);
		}
	}

	// turn the cursor on again
	displayCursor(true);

	return 0;
}


int updateDisplay(Menu::MenuNode* current) {

	int *vc = &current->vectorCursor;
	int *displayOffset = &current->displayOffset;

	// Somehow this only works properly when multiplied by -1
    // which is not the case during MenuNode use...
	*vc = (encoder->get_position()/(-4));
	if(*vc<0) {
		*vc = 0;
		encoder->set_position(0);
	}

	if(*vc>((int)current->v.size()-1)) {
		*vc = (int)current->v.size()-1;
		encoder->set_position(((int)current->v.size()-1)*-4);
	}

	// The next two if statement determine whether the display mask should be shifted
	// Execute if the screen needs to be moved down
	if(*vc>(*displayOffset+3) && *vc<(int)current->v.size()) {
		(*displayOffset)++;
		clearDisplay();

		displayMenu(current);

	}

	// Execute if the screen needs to be moved up
	if(*vc < *displayOffset) {
		(*displayOffset)--;
		clearDisplay();

		displayMenu(current);

	}

	// Place the cursor at the right position on the display
	int displayPosition[4] = {0x80, 0xC0, 0x94, 0xD4};
	lcd_write(displayPosition[*vc - *displayOffset], 0);

	return 0;

}

int writeData2File(ofstream datafile, int i) {

	clearDisplay();
	displayCursor(false);

	if (datafile.is_open()) {
		cout << "File successfully opened" << endl;
		lcd_display_string(" Writing to USB...", 2);
		datafile <<  vLengths[i]->length << ":\t";
		for (int k = 0; k < (int) vMeasurements.size(); k++) {
			datafile <<  static_cast<int>(vMeasurements[k]->force) << "\t";
		}
		datafile << "\n";
		datafile.close();
	} else {
		cout << "Error opening file!" << endl;
		lcd_display_string("Error opening file!", 2);
	}

	sleep(2);

	return 0;
}

// A function to display a binary question. The question
// is passed to the function as a string attribute and
// the user can select OK or Cancel
int prompt(string str) {

	// Clear the display. Cursor is on
	clearDisplay();
	displayCursor(true);

	// Display the binary question on the first line
	lcd_display_string(str, 1);

	// Display the options to the user
	lcd_display_string("         OK", 3);
	lcd_display_string("       Cancel", 4);

	// Start the cursor at OK
	encoder->set_position(4);

	// Variable and while loop to keep track of the user's
	// encoder movement and selection
	int cursor;
	while(button1GPIO->getValue() == LOW) {
		if (encoder->get_position() > 0) {
			cursor = 1;
			encoder->set_position(4);
		} else {
			cursor = 0;
			encoder->set_position(0);
		}
		if (cursor) {
			lcd_write(0x9C, 0);
		} else {
			lcd_write(0xDA, 0);
		}
	}

	return cursor;
}

muscleSize promptChooseMuscle() {

	// Clear the display
	clearDisplay();
	displayCursor(false);

	lcd_display_string("  Which muscle is", 1);
	lcd_display_string("    being used", 2);

	int selector;
	muscleSize muscleChoice;
	encoder->set_position(0);

	while(button1GPIO->getValue() == LOW) {

		selector = encoder->get_position()/4;

		if (selector < 1) {
			selector = 0;
			encoder->set_position(0);
			lcd_display_string("                 5mm", 4);

		}
		if (selector == 1) {
			lcd_display_string("                10mm", 4);

		}
		if (selector == 2) {
			lcd_display_string("                20mm", 4);

		}
		if (selector > 2) {
			selector = 3;
			encoder->set_position(12);
			lcd_display_string("                40mm", 4);
		}

	}

	switch (selector) {
	case 0:
		// Case 0 and case 1 (below) use the same voltage divider
		switch1GPIO->streamWrite(LOW);
		switch2GPIO->streamWrite(HIGH);
		switch3GPIO->streamWrite(LOW);

		muscleChoice = muscle5mm;
		break;
	case 1:
		switch1GPIO->streamWrite(LOW);
		switch2GPIO->streamWrite(HIGH);
		switch3GPIO->streamWrite(LOW);

		muscleChoice = muscle10mm;
		break;
	case 2:
		switch1GPIO->streamWrite(LOW);
		switch2GPIO->streamWrite(LOW);
		switch3GPIO->streamWrite(HIGH);

		muscleChoice = muscle20mm;
		break;

	case 3:
		switch1GPIO->streamWrite(HIGH);
		switch2GPIO->streamWrite(LOW);
		switch3GPIO->streamWrite(LOW);

		muscleChoice = muscle40mm;
		break;

	default:
		lcd_display_string("      Error!", 1);
		lcd_display_string(" Set to default (20mm)", 1);
		switch1GPIO->streamWrite(LOW);
		switch2GPIO->streamWrite(LOW);
		switch3GPIO->streamWrite(HIGH);
	}

	// Clear the display
	clearDisplay();

	return muscleChoice;
}

int promptInterpolatePressure() {

	clearDisplay();
	displayCursor(false);

	lcd_display_string("Select the number of", 1);
	lcd_display_string("interpolation points", 2);

	int interpolationPoints;
	encoder->set_position(0);

	while(button1GPIO->getValue() == LOW) {

		interpolationPoints = encoder->get_position()/4;

		if (interpolationPoints < 0) {
			interpolationPoints = 0;
			encoder->set_position(0);
		}
		if (interpolationPoints > 3) {
			interpolationPoints = 3;
			encoder->set_position(12);
		}

		lcd_display_int(interpolationPoints, 8);
	}

	vMeasurements.clear();

	for(unsigned int i = 0; i < vLengths.size(); i++) {
		if (i == 0) {
			vMeasurements.push_back(new data::Measurement);
			vMeasurements.back()->pressure = vLengths[i]->pressure;
			vMeasurements.back()->force = 0;
		} else {
			float spacer = (vLengths[i]->pressure - vLengths[i - 1]->pressure) / (interpolationPoints + 1);
			for (int j = 0; j <= interpolationPoints; j++) {
				vMeasurements.push_back(new data::Measurement);
				vMeasurements.back()->pressure = vLengths[i - 1]->pressure + spacer*(j + 1);
				vMeasurements.back()->force = 0;
			}
		}
	}

	// Clear the display
	clearDisplay();

	return interpolationPoints;
}

int promptMeasurementCycles() {

	clearDisplay();
	displayCursor(false);

	lcd_display_string(" Select measurement", 1);
	lcd_display_string(" cycles per length", 2);

	int measurementCycles;
	encoder->set_position(4);

	while(button1GPIO->getValue() == LOW) {

		measurementCycles = encoder->get_position()/4;

		if (measurementCycles < 1) {
			measurementCycles = 1;
			encoder->set_position(4);
		}

		lcd_display_int(measurementCycles, 8);
	}

	// Clear the display
	clearDisplay();

	return measurementCycles;
}

// This function tries to mount the USB stick. If this
// fails, the user will be asked to insert a stick.
int promptUSB() {

	// Try to mount the USB stick
	while (usb::mountUSB() < 0 ){
		// Warn user that no stick was found
		clearDisplay();
		lcd_display_string("   No USB stick!", 2);
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


int adjustPressure(float pressure) {

	clearDisplay();
	displayCursor(false);
	lcd_display_string(" Adjusting pressure", 2);
	lcd_display_string("    Please wait", 3);

	// The control function  can't regulate the pressure in the muscle
	// down to below 50mbar as the pressure sensor always shows a residual
	// pressure. Given that the user can only increment the pressure set point
	// by 50mbar, anything below this can be set to 0bar.


		// Start a thread running the control function and
		// pass it the pressure that should be adjusted to
		//startThread(controlfunc, &pressure);
		sleep(1); 	// pause, to give the thread time to initialise


		float margin;
		if (pressure < 0.15) {
			margin = 0.2;
		} else {
			margin = pressure / 20;
		}

		float error;
		int count = 1200;
		do {
			error = sqrt(pow(pressure - (pressureInBar/1000), 2));

			if (error < margin) {
				count--;
			}

		} while (error > margin || count != 0);

		cout << "New pressure reached" << endl;
		sleep(1);

		exitThread = true;


	// A short amount of time is needed to terminate the thread properly
	// otherwise the valve will move to zero
	usleep(10000);

	return 0;

}



float measureForce(muscleSize choiceOfMuscle) {

	float force;

	switch(choiceOfMuscle) {
	// The 5mm and 10mm muscles use the same voltage divider and
	// are read from the same ADC input pin
	case muscle5mm:
	case muscle10mm:
		force = readAnalog(MAX3V3);			// read the ADC value
		force = force * 1800/(pow(2,12)-1); // convert to milli-volts
		force = force * (47.5 + 57)/57;		// voltage divider
		force = force * 327/1000;			// force sensor ratio
		break;
	case muscle20mm:
		force = readAnalog(MAX6V);
		force = force * 1800/(pow(2,12)-1);
		force = force * (77.2 + 33.2)/33.2;
		force = force * 327/1000;
		break;
	case muscle40mm:
		force = readAnalog(MAX10V);
		force = force * 1800/(pow(2,12)-1);
		force = force * (91+20)/20;
		force = force * 327/1000;
		break;
	default:
		cout << "Error: unable to determine the choice of muscle being used" << endl;
		return -1;
	}

	return force;
}

int initCANbus() {

	// The bit rate is fixed at 1MHz
	int retv = system("ip link set can0 up type can bitrate 1000000");
	if (retv == -1) {
		cout << "Error: CAN bus not initialised" << endl;
	} else {
		if (WEXITSTATUS(retv) == 127) {
			cout << "Error: could not execute /bin/sh" << endl;
		} else {
			cout << "CAN bus initialised. System returned with " << WEXITSTATUS(retv) << endl;

			can0 = new CANSocket("can0");

			// Initialise the valve
			int ret = can0->initNode(0x01);
			if (ret < 0) {
				perror("CANbus");
			}
		}
	}

	return 0;
}



