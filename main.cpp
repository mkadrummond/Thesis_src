/**
 *  @file    main.cpp
 *  @author  Michael Drummond
 *  @date	 29/11/2016
 *  @version 1.0
 *
 *  @brief	 Software for the automated measurement of PMAs
 *
 *  @section DESCRIPTION
 *  A test rig for the automated measurement of the static force characteristic
 *  of PMAs
 */

#include <unistd.h>
#include <pthread.h>

#include "CAN.h"
#include "GPIO.h"
#include "menu.h"
#include "eqep.h"
#include "deviceControl.h"

using namespace std;
using namespace exploringBB; // for the GPIO buttons

void testStepResponse();

/*** GLOBAL OBJECT POINTERS ***/
GPIO 		  *button1GPIO,
			  *ledGPIO;
eQEP 		  *encoder;
LCD 		  *lcd;
Menu 		  *deviceMenu;
CANSocket 	  *can0;
DeviceControl *control;

int main() {

	if(getuid()!=0){
		cout << "You must run this program as root. Exiting." << endl;
		cout << getuid() << endl;
		return -1;
	}

	can0 = new CANSocket("can0");
    control = new DeviceControl(can0);
    cout << "RigControl instantiated" << endl;
    control->initCANbus();
	sleep(1);
	// Initialise the valve
	control->initValve();

    // Create quadrature encoder
    encoder = new eQEP(eQEP2, eQEP::eQEP_Mode_Absolute);
    encoder->set_period(10000000L); // Encoder polled every 10ms
    cout << "eQEP instantiated" << endl;

    // Create GPIO objects
    // Button:
    button1GPIO = new GPIO(88);        	//button down
    button1GPIO->setDirection(INPUT);   // button 1 is an input
    button1GPIO->setEdgeType(RISING);   //wait for rising edge
    cout << "GPIO" << endl;

	//Initialise the LCD
    lcd = new LCD();
	lcd->fd = control->initI2C();
	lcd->initLCD();
    cout << "I2C and LCD initialised" << endl;

    // Setup the menu
    deviceMenu = new Menu(encoder, button1GPIO, lcd);
    cout << "Menu instantiated" << endl;
   	deviceMenu->initMenu();
    cout << "Menu initialised" << endl;

	// Initialise the PRUs
    control->initPRU();
    cout << "PRU initialised" << endl;

	control->startThread(control);
    cout << "Control thread started" << endl;

	//testStepResponse();

	lcd->clearDisplay();
	deviceMenu->map["root"]->useNode();

return 0;
}

void testStepResponse() {
	sleep(20); // Needs time to settle after start
	int stepResponse[1000];
	float step;
	control->pressureSetPoint = 0;
	for(int i = 0; i < 1000; i++) {
		control->pressureSetPoint = 4;
		stepResponse[i] = control->pressureMeasured;
		usleep(10000);
	}
	for(int i = 0; i < 1000; i++) {
		std::cout << stepResponse[i] << std::endl;
	}
	control->pressureSetPoint = 0;
}

