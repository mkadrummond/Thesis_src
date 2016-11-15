/*
 * adc.cpp
 *
 *  Created on: 7 Nov 2016
 *      Author: mkadrummond
 */

#include"adc.h"
// Includes for the ADC input from the force sensor
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
// Linux path to read the ADC input from the force sensor
#define LDR_PATH "/sys/bus/iio/devices/iio:device0/in_voltage"



ADC::ADC() {

}

int ADC::readAnalog(int number){
	std::stringstream ss;
    ss << LDR_PATH << number << "_raw";
    std::fstream fs;
    fs.open(ss.str().c_str(), std::fstream::in);
    fs >> number;
    fs.close();
    return number;
}

