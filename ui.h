/*
 * ui.h
 *
 *  Created on: 14 Sep 2016
 *      Author: mkadrummond
 */

#ifndef UI_H_
#define UI_H_


#include "menu.h"

namespace ui {

int useRoot();

	extern Menu::MenuNode *root;
		extern Menu::MenuNode *setup;
			extern Menu::MenuNode *chooseMuscle;
			extern Menu::MenuNode *mountUSB;
			extern Menu::MenuNode *unMountUSB;
			extern Menu::MenuNode *runWarmUp;
		extern Menu::MenuNode *manual;
		extern Menu::MenuNode *automatic;
			extern Menu::MenuNode *program;
				extern Menu::MenuNode *addNewLength;
			extern Menu::MenuNode *pressurePoints;
				extern Menu::MenuNode *addNew;
			extern Menu::MenuNode *setLengths;
			extern Menu::MenuNode *startProgram;
			extern Menu::MenuNode *uploadToUSB;
		extern Menu::MenuNode *shutdownBBB;

		int initMenu();
		int createMenu();


}

#endif /* UI_H_ */
