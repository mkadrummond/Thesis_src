/*
 * menuStructure.h
 *
 *  Created on: 18 Nov 2016
 *      Author: mkadrummond
 */

#ifndef MENUSTRUCTURE_H_
#define MENUSTRUCTURE_H_

#include <string>

#include "deviceControl.h"
#include "menu.h"

extern Menu *deviceMenu;

int useRoot();
int useManual();
int useShutdown();
int useSetLengths();
int useAddNewLength();
int useAutomatic();
int useStartProgram();
int useSave();
int useLoad();
int useLength();

int prompt(std::string);
rig::muscleSize promptChooseMuscle();
int promptInterpolatePressure();
int promptMeasurementCycles();
int promptUSB();

// Backend functions
float setPressure(float*);
int setLength(int);


#endif /* MENUSTRUCTURE_H_ */
