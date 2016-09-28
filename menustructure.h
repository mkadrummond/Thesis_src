/*
 * menustructure.h
 *
 *  Created on: 14 Sep 2016
 *      Author: mkadrummond
 */

#ifndef MENUSTRUCTURE_H_
#define MENUSTRUCTURE_H_



//MenuController *controller;

Menu::MenuNode *root;
	Menu::MenuNode *setup;
		Menu::MenuNode *chooseMuscle;
		Menu::MenuNode *mountUSB;
		Menu::MenuNode *unMountUSB;
		Menu::MenuNode *runWarmUp;
	Menu::MenuNode *manual;
	Menu::MenuNode *automatic;
		Menu::MenuNode *setLengths;
			Menu::MenuNode *addNewLength;
		Menu::MenuNode *pressurePoints;
		//	Menu::MenuNode *addNew;
		//Menu::MenuNode *setLengths;
		Menu::MenuNode *startProgram;
		Menu::MenuNode *save;
		Menu::MenuNode *load;
		Menu::MenuNode *uploadToUSB;
	Menu::MenuNode *shutdownBBB;







#endif /* MENUSTRUCTURE_H_ */
