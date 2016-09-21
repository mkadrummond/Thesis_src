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

// Mounting USB
#include <sys/mount.h>

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


int manualPressure(int);
float binary2Bar(int);
//int bar2Binary(int);
int readAnalog(int);

// MenuNode use cases
int useRoot();
int useSetup();
int useMountUSB();
int useUnMountUSB();
int useManualMenuNode();
int useShutdownMenuNode();
int usePressurePoints();
int useAddNew();
int useSetPressure();
int useAutomatic();
int useStartProgram();
int useUploadToUSB();
int useChooseMuscle();

int displayMenu(MenuNode*);
int updateDisplay(MenuNode*);

int choiceOfMuscle;


// The display is only 4 lines, so if the menu list has more than 4 items
// I use an offset to allow what is actually displayed to shift up and down
// with the cursor.
//int displayOffsetPtr;
//int vectorCursor;

double pressureInBinary;
double pressureSetPoint;
double pressureInBar;

int encoderPosition;

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


	/*** Prepare the GUI menu structure ***/

	controller = new MenuController();

	// Create the menu node elements
	rootElement = new MenuNode("root");
		setup = new MenuNode("Setup");
			chooseMuscle = new MenuNode("Choose Muscle");
			mountUSB = new MenuNode("Mount USB");
			unMountUSB = new MenuNode("Unmount USB");
			runWarmUp = new MenuNode("Run warm up");
		manual = new MenuNode("Manual");
		automatic = new MenuNode("Automatic");
			pressurePoints = new MenuNode("Set pressure points");
				addNew = new MenuNode("Add new");
			setLengths = new MenuNode("Set lengths");
			startProgram = new MenuNode("Start program");
			uploadToUSB = new MenuNode("Upload data to USB");
		shutdownBBB = new MenuNode("Shutdown");

	// Create the menu structure by defining child/parent elements
	rootElement->setChild(setup);
		setup->setChild(chooseMuscle);
		setup->setChild(mountUSB);
		setup->setChild(unMountUSB);
		setup->setChild(runWarmUp);
	rootElement->setChild(manual);
	rootElement->setChild(automatic);
		automatic->setChild(pressurePoints);
			pressurePoints->setChild(addNew);
		automatic->setChild(setLengths);
		automatic->setChild(startProgram);
		automatic->setChild(uploadToUSB);
	rootElement->setChild(shutdownBBB);

	// On start the menu will be at the root node
	controller->setCurrent(rootElement);
	//std::vector<MenuNode*> v = rootElement->getVector();

	/*** Initialise the LCD  ***/

	initI2C();
	initLCD();


	// Try to mount the USB stick
	if (mount("/dev/sda1", "/mnt/usb", "vfat", NULL, NULL)) {
		cout << "Mount error: " << strerror(errno) << endl;
		lcd_display_string("USB not mounted", 2);
	} else {
		cout << "Mount successful" << endl;
		lcd_display_string("USB mounted", 2);
	}

	sleep(1);

	lcd_write(LCD_CLEARDISPLAY, 0);





	while(true) {


		if(controller->current == rootElement) {
    		int ret = useRoot();
    	}

		if(controller->current == setup) {
			int ret = useSetup();
		}

		if(controller->current == chooseMuscle) {
			int ret = useChooseMuscle();
		}

		if(controller->current == mountUSB) {
    		int ret = useMountUSB();
    	}

		if(controller->current == unMountUSB) {
    		int ret = useUnMountUSB();
    	}

		if(controller->current == manual) {
    		int ret = useManualMenuNode();
    	}

    	if(controller->current == automatic) {
    		int ret = useAutomatic();
    	}

    	if(controller->current == shutdownBBB) {
    		int ret = useShutdownMenuNode();

    		// close the program
    		return 0;
    	}

    	if(controller->current == automatic) {
    		int ret = useAutomatic();
    	}

    	if(controller->current == pressurePoints) {
    		int ret = usePressurePoints();
    	}

    	if(controller->current == uploadToUSB) {
    		int ret = useUploadToUSB();
    	}

    	if(controller->current == startProgram) {
    		int ret = useStartProgram();
    		cout << "controller current: " << controller->current << endl;
    	}
	}


return 0;
}


/*** FUNCTIONS ***/

int useRoot() {

	displayMenu(rootElement);

	while(button1GPIO->getValue() == LOW) {
		updateDisplay(controller->current);
		cout << readAnalog(1) << endl;
	}

	cout << "Button Pressed" << endl;
	controller->current = controller->current->v[controller->current->vectorCursor];
	cout << "current has changed to " << controller->current->getName() << endl;

    return 0;
}

int useSetup() {

	displayMenu(setup);

	while(button1GPIO->getValue() == LOW) {
		updateDisplay(controller->current);
	}

	cout << "Button Pressed" << endl;
	controller->current = controller->current->v[controller->current->vectorCursor];
	cout << "current has changed to " << controller->current->getName() << endl;

	//lcd_write(LCD_CLEARDISPLAY, 0);
	encoder->set_position(0);

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

	//sleep(2);

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
			break;
	}

	cout << "Button Pressed" << endl;
	controller->current = controller->current->v[controller->current->vectorCursor];
	cout << "current has changed to " << controller->current->getName() << endl;

	// turn the cursor on again
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	return 0;
}

int useMountUSB() {

	if (mount("/dev/sda1", "/mnt/usb", "vfat", NULL, NULL)) {
		if (errno == EBUSY) {
			cout << "Mountpoint busy" << endl;
		} else {
			cout << "Mount error: " << strerror(errno) << endl;
		}
	} else {
		cout << "Mount successful" << endl;
	}

	lcd_display_string(strerror(errno), 1);
	sleep(2);

	lcd_write(LCD_CLEARDISPLAY, 0);
	controller->current = setup;
	cout << "current has changed to " << controller->current->getName() << endl;
	return 0;
}

int useUnMountUSB() {

	//int status;
	//status = umount("/mnt/usb");
	if (umount("/mnt/usb")) {
		cout << "Unmount error: " << strerror(errno) << endl;
	} else {
		cout << "Unmount successful" << endl;
	}

	lcd_display_string(strerror(errno), 1);
	sleep(2);

	lcd_write(LCD_CLEARDISPLAY, 0);
	controller->current = setup;
	cout << "current has changed to " << controller->current->getName() << endl;
	return 0;

}


int useManualMenuNode() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	lcd_display_string("Set pressure", 1);

	// Return the encoder to the zero position
	encoder->set_position(0);



	startThread(controlfunc);

	// Stay in this MenuNode and do what it does until a button press
	while(button1GPIO->getValue() == LOW) {
		ledGPIO->streamWrite(HIGH);

		pressureSetPoint = encoder->get_position()*100;

		// Don't go negative
		if(pressureSetPoint < 0) {
			pressureSetPoint = 0;
		}

		lcd_display_float(pressureInBar, 3);
		lcd_display_float(pressureSetPoint, 4);

		ledGPIO->streamWrite(LOW);
	}

	// Terminate the thread
	exitThread = true;

	// Wait, to make sure thread has properly terminated
	sleep(1);

    // Return to the parent MenuNode
	controller->current = controller->current->v[0];

	// Turn the cursor back on and clear the display in preparation to show the menu
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);
	lcd_write(LCD_CLEARDISPLAY, 0);
	encoder->set_position(0);

	return 0;

}

int useAutomatic() {

	displayMenu(automatic);

    while(button1GPIO->getValue() == LOW) {
    	updateDisplay(controller->current);
    }


    sleep(0.1);

	cout << "Button Pressed" << endl;
	controller->current = controller->current->v[controller->current->vectorCursor];
	cout << "current has changed to " << controller->current->getName() << endl;



	return 0;
}

int useShutdownMenuNode() {
	cout << "run to stop" << endl;

	// Return the valve to 0
	can0->sendFrame(0x100, 0);

	can0->close_port();
	system("ifconfig can0 down");

	delete rootElement;
	delete setup;
	delete mountUSB;
	delete runWarmUp;
	delete manual;
	delete automatic;
	delete pressurePoints;
	delete setLengths;
	delete startProgram;
	delete shutdownBBB;

	prussdrv_exit ();

	//system("shutdown -h now");

	return 0;
}

int usePressurePoints() {

	displayMenu(pressurePoints);

    usleep(100000);

	//vectorCursor = (int)controller->current->v.size()-1;

	while(button1GPIO->getValue() == LOW) {
		updateDisplay(controller->current);
	}

	cout << "Button Pressed" << endl;
	controller->current = controller->current->v[controller->current->vectorCursor];
	cout << "current has changed to " << controller->current->getName() << endl;



	if(controller->current == automatic) {
		return 0;
	}
	if(controller->current == addNew) {
		useAddNew();

	} else {
		useSetPressure();
	}

	return -1;
}

int useAddNew() {


	MenuNodeFactory *NodeFactory = new MenuNodeFactory;

	// Move back up a level, because we want to edit the vector of
	// menuNodes in the level that contains the menuNode addNew and
	// not addNew itself
	controller->current = controller->current->v[0];

	stringstream menuNodeName;
    menuNodeName << "Pressure " << controller->current->vectorCursor;
    controller->current->v.insert( controller->current->v.begin() + controller->current->vectorCursor, (NodeFactory->createMenuNode(menuNodeName.str())));



	useSetPressure();


	return 0;
}

int useSetPressure() {

	controller->current = pressurePoints;

	encoder->set_position(pressureSetPoint/12.5);

	displayMenu(pressurePoints);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);


	int pressureSP;

	while(button1GPIO->getValue() == LOW) {

		pressureSP = encoder->get_position()*12.5;

		// Don't go negative
		if(pressureSP < 0) {
			pressureSP = 0;
		}

		lcd_display_float(pressureSP, controller->current->vectorCursor - controller->current->displayOffset + 5);
	}

	controller->current->v[controller->current->vectorCursor]->setPressure(pressureSP);

	// turn the cursor on
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	//clear display
	lcd_write(LCD_CLEARDISPLAY, 0);

	encoder->set_position(controller->current->vectorCursor*-4);

	cout << "Button Pressed" << endl;
	controller->current = pressurePoints;
	cout << "current has changed to " << controller->current->getName() << endl;

	return 0;
}

int useStartProgram() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	for (int i = 1; i < pressurePoints->v.size() - 1; i++) {
		lcd_display_string(pressurePoints->v[i]->getName(), 1);
		lcd_display_float(pressurePoints->v[i]->getPressure(), 5);
		pressureSetPoint = pressurePoints->v[i]->getPressure();
		startThread(controlfunc);
		sleep(5);
		int force = readAnalog(choiceOfMuscle);
		pressurePoints->v[i]->setForce(force);
		exitThread = true;
		lcd_write(LCD_CLEARDISPLAY, 0);
	}

	// Bring the valve to it's centred position
	can0->sendFrame(0x100, 0);

	lcd_display_string("Measurement success", 1);

	// Bring the valve to it's centred position
	can0->sendFrame(0x100, 0);
	sleep(2);

	controller->current = automatic;

	// turn the cursor on
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	return 0;
}

int useUploadToUSB() {

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	ofstream datafile;
	datafile.open("/mnt/usb/data.txt", ios::out | ios::app | ios::binary);
	if (datafile.is_open()) {
		cout << "File successfully opened" << endl;
		lcd_display_string("Writing to file...", 1);
		for (int i = 1; i < (int)pressurePoints->v.size()-1; i++) {
			datafile << pressurePoints->v[i]->getName() << " ";
			datafile << pressurePoints->v[i]->getPressure() << " ";
			datafile << pressurePoints->v[i]->getForce() << "\n";
		}
		datafile.close();
	} else {
		cout << "Error opening file!" << endl;
		lcd_display_string("Error opening file!", 2);
	}

	// Allow the user to see that the write has been successful
	sleep(1);

	controller->current = controller->current->v[0];
	cout << "current has changed to " << controller->current->getName() << endl;


	return 0;
}

void *controlfunc(void *parm)
{
	float pressureOutput;

	float dt;
    double max;
    double min;
    float Kp;
    float Kd;
    float Ki;

    dt = 0.01;
    max = 2;
    min = -2;
    Kp = 1;
    Kd = 0;
    Ki = 0.8;

	PID *pidControl = new PID(dt, max, min, Kp, Kd, Ki);

	while (exitThread == false) {


		// Wait for event completion from PRU
		int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);

		prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

		// read the adc
		pressureInBinary = readAnalog(0);

 		pressureInBar = binary2Bar(pressureInBinary);

		pressureOutput = pidControl->calculate( pressureSetPoint, pressureInBar );

		can0->sendFrame(0x100, pressureOutput);
	}

	delete pidControl;
	cout << "exiting thread!" << endl;
	exitThread = false;
	return NULL;
}

int manualPressure(int x) {
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

int displayMenu(MenuNode* current) {

	int *displayOffset = &current->displayOffset;

	//encoder->set_position(current->vectorCursor/4);

	// Clear the display
	lcd_write(LCD_CLEARDISPLAY, 0);

	// turn the cursor off because it's distracting
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);

	for(int i = 0; i < (int)current->v.size() && i<4; i++) {
		if(*displayOffset+i == 0 && current != rootElement) {
			lcd_display_string("Back", 1);
		} else {
			lcd_display_string(current->v[*displayOffset + i]->getName(), i+1);
		}
	}
	if(current == pressurePoints) {
		for(int i = 0; i < (int)current->v.size()-1 && i<4; i++) {
			if(*displayOffset+i == 0 || *displayOffset+i == (int)current->v.size()-1) {
			} else {
				lcd_display_float(current->v[*displayOffset + i]->getPressure(), i+5);
			}
		}
	}

	// turn the cursor on again
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);

	return 0;
}

int updateDisplay(MenuNode* current) {

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
		lcd_write(LCD_CLEARDISPLAY, 0);

		displayMenu(current);
	}

	// Execute if the screen needs to be moved up
	if(*vc < *displayOffset) {
		(*displayOffset)--;
		lcd_write(LCD_CLEARDISPLAY, 0);

		displayMenu(current);
	}

	// Place the cursor at the right position on the screen
	int displayPosition[4] = {0x80, 0xC0, 0x94, 0xD4};
	lcd_write(displayPosition[*vc - *displayOffset], 0);


	return 0;
}

