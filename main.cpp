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

// Includes for the ADC input from the force sensor
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
// Linux path to read the ADC input from the force sensor
#define LDR_PATH "/sys/bus/iio/devices/iio:device0/in_voltage"

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

// PID controller
// from: https://gist.github.com/bradley219/5373998
#include "pid.h"

// Threads
#include "thread.h"

//PRU
#include <stdlib.h>
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#define PRU_NUM 0   	// using PRU0 for the timer
#define PRU_BIN_ADDR 	"/home/mkadrummond/pru/timer.bin"	//

// Menu structure
#include "menustructure.h"
#include <sstream> //required for the terminal nodes
//#include "ui.h"


// Mounting USB
#include <sys/mount.h>
#include "usb.h"

//using namespace exploringBB; // for the gpio buttons


#include <stdio.h>
#include <iostream>
using namespace std;
using namespace exploringBB; // for the gpio buttons




//global pointers
// GPIOs
GPIO 	*button1GPIO,
		*ledGPIO,
		*switch1GPIO,
		*switch2GPIO,
		*switch3GPIO;

// CAN-bus
CANSocket *can0;

// Rotary encoder
eQEP *encoder;

// PID control
PID *pidControl;


//int manualPressure(int);
float binary2Bar(int);
float binary2Newton(int);
int readAnalog(int);

// MenuNode use cases
int useRoot();
int useSetup();
int useMountUSB();
int useUnMountUSB();
int useManual();
int useShutdown();
int useSetLengths();
	int useAddNewLength();
		int useSetLength();
int usePressurePoints();
//int useAddNew();
int useSetPressure();
int useAutomatic();
int useStartProgram();
int useSave();
int useLoad();
int useUploadToUSB();
int useChooseMuscle();

float setPressure(float);

int displayMenu(Menu::MenuNode*);
int updateDisplay(Menu::MenuNode*);

void debug();

int choiceOfMuscle;

double pressureInBinary;
float pressureSetPoint;
double pressureInBar;


bool exitThread;

void *controlfunc(void*);

int main() {

	if(getuid()!=0){
		cout << "You must run this program as root. Exiting." << endl;
		cout << getuid() << endl;
		return -1;
	}

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

	system("ip link set can0 up type can bitrate 1000000"); // The bit rate is fixed at 1MHz


	can0 = new CANSocket("can0");
	sleep(1);

	// Initialise the valve
	int in = can0->initNode(0x01);
	if (in < 0) {
		cout << "Error: node initialisation unsuccessful; writing to CAN-bus failed." << endl;
	}

	// Bring the valve to it's centred position
	can0->sendFrame(0x100, 0);

	/*** Prepare the rotary encoder, buttons and LEDs ***/

    // create quadrature encoder
    encoder = new eQEP(eQEP2, eQEP::eQEP_Mode_Absolute);
    encoder->set_period(100000000L);
    //vectorCursor = encoder->get_position();


    // Create GPIO objects
    // Button:
    button1GPIO = new GPIO(88);        	//button down
    button1GPIO->setDirection(INPUT);   // button 1 is an input
    button1GPIO->setEdgeType(RISING);   	//wait for rising edge

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

    switch1GPIO->streamWrite(HIGH);
    switch2GPIO->streamWrite(LOW);
    switch3GPIO->streamWrite(LOW);

    choiceOfMuscle = 1;

    /*** Initialise the LCD  ***/
	initI2C();
	initLCD();

	/*** Mount USB stick if available ***/
	usb::mountUSB();


	/*** Prepare the GUI menu structure ***/

	// Create the menu node elements
	root = new Menu::MenuNode("root", useRoot);
		setup = new Menu::MenuNode("Setup", root, useSetup);
			chooseMuscle = new Menu::MenuNode("Choose Muscle", setup, useChooseMuscle);
			mountUSB = new Menu::MenuNode("Mount USB", setup, useMountUSB);
			unMountUSB = new Menu::MenuNode("Unmount USB", setup, useUnMountUSB);
			runWarmUp = new Menu::MenuNode("Run warm up", setup);
		manual = new Menu::MenuNode("Manual", root, useManual);
		automatic = new Menu::MenuNode("Automatic", root, useAutomatic);
			setLengths = new Menu::MenuNode("Set lengths", automatic, useSetLengths);
				addNewLength = new Menu::MenuNode("Add new length", setLengths, useAddNewLength);
			pressurePoints = new Menu::MenuNode("Pressure points", automatic, usePressurePoints);
				//addNew = new Menu::MenuNode("Add new", pressurePoints, useAddNew);
			//setLengths = new Menu::MenuNode("Set lengths", automatic);
			startProgram = new Menu::MenuNode("Start program", automatic, useStartProgram);
			save = new Menu::MenuNode("Save program", automatic, useSave);
			load = new Menu::MenuNode("Load program", automatic, useLoad);
			uploadToUSB = new Menu::MenuNode("Upload data to USB", automatic, useUploadToUSB);
		shutdownBBB = new Menu::MenuNode("Shutdown", root, useShutdown);


	clearDisplay();

	// On start the menu will be at the root node
	Menu::current = root;
	Menu::current->useNode();

return 0;
}


/*** FUNCTIONS ***/

int useRoot() {

	while(true) {

		displayMenu( root);

		while(button1GPIO->getValue() == LOW) {
			updateDisplay(Menu::current);
		}

		cout << "Button Pressed" << endl;
		Menu::current = Menu::current->v[Menu::current->vectorCursor];
		cout << "current has changed to " << Menu::current->getName() << endl;

		Menu::current->useNode();
	}

    return -1;
}

int useSetup() {

	clearDisplay();

	while(true) {

		displayMenu( setup);

		while(button1GPIO->getValue() == LOW) {
			updateDisplay(Menu::current);
		}

		cout << "Button Pressed" << endl;
		Menu::current = Menu::current->v[Menu::current->vectorCursor];
		cout << "current has changed to " << Menu::current->getName() << endl;

		encoder->set_position(0);

		Menu::current->useNode();

	}

	return 0;
}

int useChooseMuscle() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	// Return the encoder to the zero position
	encoder->set_position((choiceOfMuscle-1)*4);

	lcd_display_string("Choice of Muscle: ", 2);

	sleep(1);

	while(button1GPIO->getValue() == LOW) {

		choiceOfMuscle = encoder->get_position()/4 + 1;

		// There are three muscle types, so choiceOfMuscle
		// can only take one of three values
		if (choiceOfMuscle < 1) {
			choiceOfMuscle = 1;
			encoder->set_position(0);
		}
		if (choiceOfMuscle > 3) {
			choiceOfMuscle = 3;
			encoder->set_position(8);
		}

		lcd_display_int(choiceOfMuscle, 8);

	}

	switch (choiceOfMuscle) {
	case 1:
		switch1GPIO->streamWrite(HIGH);
		switch2GPIO->streamWrite(LOW);
		switch3GPIO->streamWrite(LOW);
		break;
	case 2:
		switch1GPIO->streamWrite(LOW);
		switch2GPIO->streamWrite(HIGH);
		switch3GPIO->streamWrite(LOW);
		break;
	case 3:
		switch1GPIO->streamWrite(LOW);
		switch2GPIO->streamWrite(LOW);
		switch3GPIO->streamWrite(HIGH);
		break;
	default:
		lcd_display_string("Error", 1);
		lcd_display_string("Set to default (1)", 1);
		switch1GPIO->streamWrite(HIGH);
		switch2GPIO->streamWrite(LOW);
		switch3GPIO->streamWrite(LOW);
	}

	cout << "Button Pressed" << endl;
	Menu::current = Menu::current->v[Menu::current->vectorCursor];
	cout << "current has changed to " << Menu::current->getName() << endl;

	// turn the cursor on again
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	return 0;
}

int useMountUSB() {

	clearDisplay();

	int retval = usb::mountUSB();
	switch (retval) {
	case 0:
		lcd_display_string("USB stick mounted", 1);
		break;
	case -1:
		lcd_display_string("Mount point busy", 1);
		break;
	case -2:
		lcd_display_string("Mount error", 1);
		break;
	}

	sleep(1);

	Menu::current = setup;

	return 0;
}

int useUnMountUSB() {

	clearDisplay();

	int retval = usb::unmountUSB();
	switch (retval) {
	case 0:
		lcd_display_string("USB stick unmounted", 1);
		break;
	case -1:
		lcd_display_string("Unmount error", 1);
		break;
	}

	sleep(1);

	Menu::current =  setup;

	return 0;

}


int useManual() {

	clearDisplay();

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	lcd_display_string("Set pressure", 1);

	// Return the encoder to the zero position
	encoder->set_position(0);

	setPressure(0);

	startThread(controlfunc, &Menu::current->pressure);

	// Stay in this MenuNode and do what it does until a button press
	while(button1GPIO->getValue() == LOW) {
		ledGPIO->streamWrite(HIGH);

		pressureSetPoint = static_cast<float>(encoder->get_position())*12.5;

		// Don't go negative
		if(pressureSetPoint < 0) {
			pressureSetPoint = 0;
		}


		lcd_display_float(static_cast<float>(pressureInBar)/1000, 3);
		lcd_display_float(pressureSetPoint/1000, 4);

		ledGPIO->streamWrite(LOW);

		//float force = readAnalog(choiceOfMuscle)*(111/20)*(40/91);
		float force = readAnalog(choiceOfMuscle)*2.43956;
		//*2.43956
		cout << force << endl;
	}

	// Terminate the thread
	exitThread = true;

	// Wait, to make sure thread has properly terminated
	sleep(1);

    // Return to the parent MenuNode
	Menu::current = Menu::current->v[0];

	// Turn the cursor back on and clear the display in preparation to show the menu
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);
	lcd_write(LCD_CLEARDISPLAY, 0);
	encoder->set_position(0);

	return 0;

}

int useAutomatic() {

	while(true) {

		displayMenu( automatic);

		while(button1GPIO->getValue() == LOW) {
			updateDisplay(Menu::current);
		}

		Menu::current = Menu::current->v[Menu::current->vectorCursor];

		Menu::current->useNode();
	}

	return -1;
}

int useShutdown() {
	cout << "run to stop" << endl;

	// Return the valve to 0
	can0->sendFrame(0x100, 0);

	can0->close_port();
	system("ifconfig can0 down");

	delete  root;
	delete  setup;
	delete  mountUSB;
	delete  runWarmUp;
	delete  manual;
	delete  automatic;
	delete	addNewLength;
	delete  pressurePoints;
	delete  setLengths;
	delete  startProgram;
	delete  shutdownBBB;

	prussdrv_exit ();

	system("shutdown -h now");

	return 0;
}

int useSetLengths() {

	while(true) {
		displayMenu(setLengths);

		usleep(100000);

		//vectorCursor = (int)Menu::current->v.size()-1;

		while(button1GPIO->getValue() == LOW) {
			updateDisplay(setLengths);
		}

		cout << "Button Pressed" << endl;
		Menu::current = Menu::current->v[Menu::current->vectorCursor];
		cout << "current has changed to " << Menu::current->getName() << endl;

		Menu::current->useNode();

	}
	return -1;
}

int useAddNewLength() {

	Menu::MenuNodeFactory *NodeFactory = new Menu::MenuNodeFactory;

	// Move back up a level, because we want to edit the vector of
	// menuNodes in the level that contains the menuNode addNew and
	// not addNew itself
	Menu::current = Menu::current->v[0];

	stringstream menuNodeName;
    menuNodeName << "Length " << Menu::current->vectorCursor;
    setLengths->v.insert( Menu::current->v.begin() + Menu::current->vectorCursor, (NodeFactory->createMenuNode(menuNodeName.str(), useSetPressure)));

    cout << "pressure points: " << pressurePoints->v.size() << endl;

    if (pressurePoints->v.size() > 1) {
		// Create two pressure points for every length created
    	stringstream pressureName1;
		pressureName1 << "Pressure " << (setLengths->vectorCursor - 1) * 2;
		pressurePoints->v.push_back(NodeFactory->createMenuNode(pressureName1.str()));
		stringstream pressureName2;
		pressureName2 << "Pressure " << setLengths->vectorCursor * 2 - 1;
		pressurePoints->v.push_back(NodeFactory->createMenuNode(pressureName2.str()));
    } else {
    	// Just create one, as it's the first one
    	stringstream pressureName;
    	pressureName << "Pressure " << setLengths->vectorCursor;
		pressurePoints->v.push_back(NodeFactory->createMenuNode(pressureName.str()));
    }

    // Menu::current becomes length X, the useNode of which points to useSetPressure
	Menu::current = Menu::current->v[Menu::current->vectorCursor];
	cout << "current has changed to " << Menu::current->getName() << endl;

	// Move on to useSetPressure()
	Menu::current->useNode();

	return 0;
}

float setPressure(float pressure) {

	usleep(100000);

	//float *pres = &Menu::current->pressure;
	//cout << pres << endl;

	encoder->set_position(static_cast<int>(pressure*80));

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	clearDisplay();

	lcd_display_string("Set pressure", 2);

	startThread(controlfunc, &pressure);

	while(button1GPIO->getValue() == LOW) {


		pressure = static_cast<float>(encoder->get_position())/80;

		// Don't go negative
		if(pressure < 0) {
			pressure = 0;
		}

		lcd_display_float(pressure, 6);
		lcd_display_float(static_cast<float>(pressureInBar)/1000, 7);

	}

	// Terminate the thread
	exitThread = true;

	// Wait, to make sure thread has properly terminated
	//sleep(1);

	// Edit the pressure value of the correct pressure point
	int insertLoc = setLengths->vectorCursor * 2 -1;
	if(pressurePoints->v.size() > 2 && insertLoc > 1) {
		cout << "pressure points: " << pressurePoints->v.size() << endl;
		pressurePoints->v[insertLoc]->pressure = pressure;
		pressurePoints->v[insertLoc - 1]->pressure = (pressurePoints->v[insertLoc]->pressure + pressurePoints->v[insertLoc - 2]->pressure)/2;
	} else {
		pressurePoints->v[insertLoc]->pressure = pressure;
	}

	debug();

	// turn the cursor on
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	//clear display
	lcd_write(LCD_CLEARDISPLAY, 0);

	//encoder->set_position(Menu::current->vectorCursor*-4);

	cout << "Button Pressed" << endl;
	//Menu::current = setLengths;
	cout << "current has changed to " << Menu::current->getName() << endl;

	useSetLength();

	return 0;
}




int useSetPressure() {

	usleep(100000);

	float *pres = &Menu::current->pressure;
	cout << pres << endl;

	encoder->set_position(static_cast<int>((*pres)*80));

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	lcd_display_string("Set pressure", 2);

	startThread(controlfunc, &Menu::current->pressure);

	while(button1GPIO->getValue() == LOW) {

		//cout << *pres << endl;
		*pres = static_cast<float>(encoder->get_position())/80;

		// Don't go negative
		if(*pres < 0) {
			*pres = 0;
		}

		lcd_display_float(*pres, 6);
		lcd_display_float(static_cast<float>(pressureInBar)/1000, 7);

	}

	// Terminate the thread
	exitThread = true;

	// Wait, to make sure thread has properly terminated
	//sleep(1);

	// Edit the pressure value of the correct pressure point
	int insertLoc = setLengths->vectorCursor * 2 -1;
	if(pressurePoints->v.size() > 2 && insertLoc > 1) {
		cout << "pressure points: " << pressurePoints->v.size() << endl;
		pressurePoints->v[insertLoc]->pressure = *pres;
		pressurePoints->v[insertLoc - 1]->pressure = (pressurePoints->v[insertLoc]->pressure + pressurePoints->v[insertLoc - 2]->pressure)/2;
	} else {
		pressurePoints->v[insertLoc]->pressure = *pres;
	}

	debug();

	// turn the cursor on
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	//clear display
	lcd_write(LCD_CLEARDISPLAY, 0);

	//encoder->set_position(Menu::current->vectorCursor*-4);

	cout << "Button Pressed" << endl;
	//Menu::current = setLengths;
	cout << "current has changed to " << Menu::current->getName() << endl;

	useSetLength();

	return 0;
}



int useSetLength() {

	encoder->set_position(Menu::current->length*4);

	//Menu::current = pressurePoints;

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	lcd_display_string("Set length", 2);
	cout << Menu::current->getName() << endl;

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	usleep(100000);

	while(button1GPIO->getValue() == LOW) {

		Menu::current->length = encoder->get_position()/4;

		// Don't go negative
		if(Menu::current->length < 0) {
			Menu::current->length = 0;
		}

		lcd_display_int(Menu::current->length, 6);
	}

	// turn the cursor on
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	//clear display
	lcd_write(LCD_CLEARDISPLAY, 0);

	cout << "Button Pressed" << endl;
	Menu::current =  setLengths;
	cout << "current has changed to " << Menu::current->getName() << endl;

	return 0;
}

int usePressurePoints() {

		displayMenu(pressurePoints);

		usleep(100000);

		while(button1GPIO->getValue() == LOW) {
			updateDisplay(Menu::current);
		}

		cout << "Button Pressed" << endl;
		Menu::current = automatic;
		cout << "current has changed to " << Menu::current->getName() << endl;

	return 0;
}
/*
int useAddNew() {


	Menu::MenuNodeFactory *NodeFactory = new Menu::MenuNodeFactory;

	// Move back up a level, because we want to edit the vector of
	// menuNodes in the level that contains the menuNode addNew and
	// not addNew itself
	Menu::current = Menu::current->v[0];

	stringstream menuNodeName;
    menuNodeName << "Pressure " << Menu::current->vectorCursor;
    Menu::current->v.insert( Menu::current->v.begin() + Menu::current->vectorCursor, (NodeFactory->createMenuNode(menuNodeName.str(), useSetPressure)));

	useSetPressure();


	return 0;
}
*/


int useStartProgram() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);


	for (int i = 1; i < setLengths->v.size() - 1; i++) {

		Menu::current = setLengths->v[i];

		lcd_display_string("Unscrew the bolts", 1);
		lcd_display_string("         OK", 3);

		while(button1GPIO->getValue() == LOW) {
		}

		lcd_write(LCD_CLEARDISPLAY, 0);

		lcd_display_string("Adjusting length...", 1);

		startThread(controlfunc, &Menu::current->pressure);

		lcd_write(LCD_CLEARDISPLAY, 0);
		lcd_display_string("Fasten the muscle", 1);
		lcd_display_string("         OK", 3);

		while(button1GPIO->getValue() == LOW) {
		}

		exitThread = true;

		lcd_write(LCD_CLEARDISPLAY, 0);

		int j = i * 2 - 1;
		while (j < pressurePoints->v.size()) {

			//pressureSetPoint = pressurePoints->v[i]->pressure;
			Menu::current = pressurePoints->v[j];

			lcd_display_string(Menu::current->getName(), 1);
			lcd_display_float(Menu::current->pressure, 5);

			startThread(controlfunc, &Menu::current->pressure);
			//sleep(5);
			float force;
			int count = 120;
			while (count > 0) {
				force = readAnalog(choiceOfMuscle);
				switch(choiceOfMuscle) {
				case 1:
					force = binary2Newton(force);
					break;
				case 2:
					force = force*(47.5+57)/57;
					break;
				case 3:
					force = force*(77.2+33.2)/33.2;
					break;
				default:
					cout << "Error: unable to determine the choice of muscle being used" << endl;
					return -1;
				}

				lcd_display_float(pressureInBar/1000, 6);
				lcd_display_int(static_cast<int>(force), 8);
				count--;
			}

			pressurePoints->v[j]->force = force;

			exitThread = true;
			lcd_write(LCD_CLEARDISPLAY, 0);
			j++;
		}

		ofstream datafile;
		datafile.open("/mnt/usb/data.txt", ios::out | ios::app);
		if (datafile.is_open()) {
			cout << "File successfully opened" << endl;
			lcd_display_string("Writing to file...", 1);
			datafile << "L\t";
			datafile << "P\t";
			datafile << "F\t\n";
			for (int k = 1; k < (int) pressurePoints->v.size(); k++) {
				datafile <<  setLengths->v[i]->length << "\t";
				datafile <<  pressurePoints->v[k]->pressure << "\t";
				datafile <<  static_cast<int>(pressurePoints->v[k]->force) << "\t\n";
			}
			datafile << "\n";
			datafile.close();
		} else {
			cout << "Error opening file!" << endl;
			lcd_display_string("Error opening file!", 2);
		}

		sleep(1);

	}

	lcd_display_string("Measurement success", 1);

	// Bring the valve to it's centred position
	can0->sendFrame(0x100, 0);
	sleep(2);

	Menu::current =  automatic;

	// turn the cursor on
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	return 0;
}

int useSave() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	ofstream programFile;
	programFile.open("/mnt/usb/program.txt", ios::out | ios::app);
	if (programFile.is_open()) {
		cout << "File successfully opened" << endl;
		lcd_display_string("Writing to file...", 1);
		// Total number of Lengths; helpful for when we want to load the program later:
		programFile << setLengths->v.size() - 2 << "\n";
		for(int i = 1; i < setLengths->v.size() - 1; i++) {
			programFile << setLengths->v[i]->name << "\t";
			programFile << setLengths->v[i]->pressure << "\t";
			programFile << setLengths->v[i]->length << "\t\n";
		}

		programFile.close();
	} else {
		cout << "Error opening file!" << endl;
		lcd_display_string("Error opening file!", 2);
	}

	sleep(1);

	Menu::current = automatic;
	cout << "current has changed to " << Menu::current->getName() << endl;

	return 0;
}

int useLoad() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	Menu::MenuNodeFactory *NodeFactory = new Menu::MenuNodeFactory;






	ifstream programFile("/mnt/usb/program.txt", ios::in);

	int n;
	int datumNumber;
	float pressure;
	int length;

	if (programFile >> n) {
		cout << n << endl;
		while (n > 0) {
			if (programFile >> datumNumber >> pressure >> length) {

				stringstream menuNodeName;
				menuNodeName << "Length " << datumNumber;
				setLengths->v.insert( setLengths->v.end() - 1, (NodeFactory->createMenuNode(menuNodeName.str(), useSetPressure)));
				setLengths->v.end()[-2]->pressure = pressure;
				setLengths->v.end()[-2]->length = length;


				if (pressurePoints->v.size() > 1) {
					// Create two pressure points for every length created
					stringstream pressureName1;
					pressureName1 << "Pressure " << (datumNumber - 1) * 2;
					pressurePoints->v.push_back(NodeFactory->createMenuNode(pressureName1.str()));
					pressurePoints->v.back()->pressure = (pressure + pressurePoints->v.back()->pressure)/2;
					stringstream pressureName2;
					pressureName2 << "Pressure " << datumNumber * 2 - 1;
					pressurePoints->v.push_back(NodeFactory->createMenuNode(pressureName2.str()));
					pressurePoints->v.back()->pressure = pressure;
				} else {
					// Just create one, as it's the first one
					stringstream pressureName;
					pressureName << "Pressure " << datumNumber;
					pressurePoints->v.push_back(NodeFactory->createMenuNode(pressureName.str()));
					pressurePoints->v.back()->pressure = pressure;
				}


			} else {
				cout << "Unable to open file" << endl;
			}
		n--;
		}





	} else {
		cout << "Unable to open file" << endl;
	}
	programFile.close();


	Menu::current = automatic;
	cout << "current has changed to " << Menu::current->getName() << endl;


	return 0;
}

int useUploadToUSB() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	ofstream datafile;
	datafile.open("/mnt/usb/data.txt", ios::out | ios::app);
	if (datafile.is_open()) {
		cout << "File successfully opened" << endl;
		lcd_display_string("Writing to file...", 1);
		for (int i = 1; i < (int) pressurePoints->v.size()-1; i++) {
			datafile <<  pressurePoints->v[i]->getName() << " ";
			datafile <<  pressurePoints->v[i]->pressure << " ";
			datafile <<  pressurePoints->v[i]->force << "\n";
		}
		datafile.close();
	} else {
		cout << "Error opening file!" << endl;
		lcd_display_string("Error opening file!", 2);
	}

	// Allow the user to see that the write has been successful
	sleep(1);

	Menu::current = Menu::current->v[0];
	cout << "current has changed to " << Menu::current->getName() << endl;

	return 0;
}

void *controlfunc(void *args)
{
	float pressureOutput;
	float dt = 0.01;
    double max = 2;
    double min = -2;
    float Kp = 1;
    float Kd = 0;
    float Ki = 0.8;

	PID *pidControl = new PID(dt, max, min, Kp, Kd, Ki);

	float *pres = static_cast<float*>(args);

	while (exitThread == false) {

		// Wait for event completion from PRU
		int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);

		prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

		// read the adc
		pressureInBinary = readAnalog(0);

 		pressureInBar = binary2Bar(pressureInBinary);

		//pressureOutput = pidControl->calculate( pressureSetPoint, pressureInBar );
 		pressureOutput = pidControl->calculate( (*pres)*1000, pressureInBar );

		can0->sendFrame(0x100, pressureOutput);
	}

	delete pidControl;

	exitThread = false;

	return 0;
}




/*
void *controlfunc(void *args)
{
	float pressureOutput;
	float dt = 0.01;
    double max = 2;
    double min = -2;
    float Kp = 1;
    float Kd = 0;
    float Ki = 0.8;

	PID *pidControl = new PID(dt, max, min, Kp, Kd, Ki);

	float *pres = &Menu::current->pressure;

	while (exitThread == false) {

		// Wait for event completion from PRU
		int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);

		prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

		// read the adc
		pressureInBinary = readAnalog(0);

 		pressureInBar = binary2Bar(pressureInBinary);

		//pressureOutput = pidControl->calculate( pressureSetPoint, pressureInBar );
 		pressureOutput = pidControl->calculate( (*pres)*1000, pressureInBar );

		can0->sendFrame(0x100, pressureOutput);
	}

	delete pidControl;

	exitThread = false;

	return 0;
}
*/
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

float binary2Newton(int adcValue) {
	// The read value into millivolts: *1800/(2^12-1)
	// voltage divider: *(91+20)/20
	// The two together: (1800/(2^12-1))*(91+20)/20 = 2.43956;
	float forceAsVoltage = adcValue*1.2238742;

	return forceAsVoltage*327/1000;
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
	lcd_write(LCD_CLEARDISPLAY, 0);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	for(int i = 0; i < (int)current->v.size() && i<4; i++) {
		if(*displayOffset+i == 0 && current !=  root) {
			lcd_display_string("Back", 1);
		} else {
			lcd_display_string(current->v[*displayOffset + i]->getName(), i+1);
		}
	}

	if(current == pressurePoints) {
		for(int i = 0; i < current->v.size() && i < 4; i++) {
			cout << "i: " << i << endl;
			if(*displayOffset+i == 0) {
			} else {
				lcd_display_float(current->v[*displayOffset + i]->pressure, i+5);
			}
		}
	}

	// turn the cursor on again
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	return 0;
}

int updateDisplay(Menu::MenuNode* current) {


	int *vc = &current->vectorCursor;

	int *displayOffset = &current->displayOffset;


	//cout << "root vc: " << root->vectorCursor << endl;
	//cout << "automatic vc: " << automatic->vectorCursor << endl;

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
		lcd_write(LCD_CLEARDISPLAY, 0);

		displayMenu(current);
	}
	// Execute if the screen needs to be moved up
	if(*vc < *displayOffset) {
		(*displayOffset)--;
		lcd_write(LCD_CLEARDISPLAY, 0);

		displayMenu(current);
	}

	// Place the cursor at the right position on the display
	int displayPosition[4] = {0x80, 0xC0, 0x94, 0xD4};
	//out << "vc: " << *vc << endl;
	//cout << "do: " << *displayOffset << endl;
	lcd_write(displayPosition[*vc - *displayOffset], 0);


	return 0;
}

void debug() {
	cout << "yo" << endl;
}

