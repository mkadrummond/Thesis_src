/*
 * menustructure.h
 *
 *  Created on: 14 Sep 2016
 *      Author: mkadrummond
 */

#ifndef MENUSTRUCTURE_H_
#define MENUSTRUCTURE_H_


MenuController *controller;

MenuNode *rootElement;
	MenuNode *setup;
		MenuNode *mountUSB;
		MenuNode *unMountUSB;
		MenuNode *runWarmUp;
	MenuNode *manual;
	MenuNode *automatic;
		MenuNode *pressurePoints;
			MenuNode *addNew;
		MenuNode *setLengths;
		MenuNode *startProgram;
		MenuNode *uploadToUSB;
	MenuNode *shutdownBBB;




#endif /* MENUSTRUCTURE_H_ */
