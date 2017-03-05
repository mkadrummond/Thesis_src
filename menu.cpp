/**
 * @file 	menu.cpp
 * @author 	Michael Drummond
 * @version	1.0
 *
 * @section DESCRIPTION
 *
 */

#include "menu.h"

#include "LCD.h"
#include "menuStructure.h"

using namespace std;

MenuNode *current;

/*** Class MenuNode: ***/

MenuNode::MenuNode(std::string n, int (*pt2Func)())
	:	name(n),
		displayOffset(0),
		vectorCursor(0),
		useNode(pt2Func)
		{
		}

MenuNode::MenuNode (std::string n, MenuNode *parent, int (*pt2Func)())
	:	name(n),
		displayOffset(0),
		vectorCursor(0),
		useNode(pt2Func)
		{
			parent->setChild(this);
		}

//Member Functions
void MenuNode::setChild(MenuNode *child) {
	child->v.push_back(this);
	this->v.push_back(child);
}

std::string MenuNode::getName() {
	return this->name;
}

void MenuNode::setName(std::string name) {
	this->name = name;
}

//int MenuNode::passPtr(int (*pt2Func)()) {
//	return pt2Func();
//}
int MenuNode::*useNode() {
	return 0;
}

// Destructor
MenuNode::~MenuNode() {
}


/*** Class MenuNodeFactory ***/
MenuNode* MenuNodeFactory::createMenuNode(std::string name, int (*pt2Func)()) {
	return new MenuNode(name, pt2Func);
}


//Menu::Menu(eQEP *encoder, exploringBB::GPIO *button, DeviceControl *control)
Menu::Menu(eQEP *encoder, exploringBB::GPIO *button, LCD *lcd)
	:	encoder(encoder),
		button(button),
		lcd(lcd),
		current(NULL)
		{

		}

Menu::~Menu() {};

std::unordered_map<std::string, MenuNode*> Menu::initMenu() {

	// Create the menu node elements
	MenuNode *root = new MenuNode("root", useRoot);
		MenuNode *manual = new MenuNode(" Manual", root, useManual);
		MenuNode *automatic = new MenuNode(" Automatic", root, useAutomatic);
			MenuNode *setLengths = new MenuNode(" Set lengths", automatic, useSetLengths);
				MenuNode *addNewLength = new MenuNode(" Add new length", setLengths, useAddNewLength);
			MenuNode *startProgram = new MenuNode(" Start program", automatic, useStartProgram);
			MenuNode *save = new MenuNode(" Save program", automatic, useSave);
			MenuNode *load = new MenuNode(" Load program", automatic, useLoad);
		MenuNode *shutdownBBB = new MenuNode(" Shutdown", root, useShutdown);

		this->map.insert({"root", root});
		this->map.insert({"manual", manual});
		this->map.insert({"automatic", automatic});

		this->map.insert({"setLengths", setLengths});
		this->map.insert({"addNewLength", addNewLength});
		this->map.insert({"startProgram", startProgram});
		this->map.insert({"save", save});
		this->map.insert({"load", load});
		this->map.insert({"shutdown", shutdownBBB});

		this->current = root;
		return map;
}

int Menu::displayMenu(MenuNode* current) {

	int *displayOffset = &current->displayOffset;
	int *vc = &current->vectorCursor;

	// This makes sure that the cursor is back at the same
	// position in the menu when one returns to a menuNode
	this->encoder->set_position((*vc)*-4);

	// Clear the display
	this->lcd->clearDisplay();

	// turn the cursor off because it's distracting
	this->lcd->displayCursor(false);

	for(int i = 0; i < (int)current->v.size() && i<4; i++) {
		if(*displayOffset+i == 0 && current !=  Menu::map["root"]) {
			this->lcd->lcd_display_string(" Back", 1);
		} else {
			this->lcd->lcd_display_string(current->v[*displayOffset + i]->getName(), i+1);
		}
	}

	// turn the cursor on again
	this->lcd->displayCursor(true);

	return 0;
}


int Menu::updateDisplay(MenuNode* current) {

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
		this->lcd->clearDisplay();

		displayMenu(current);

	}

	// Execute if the screen needs to be moved up
	if(*vc < *displayOffset) {
		(*displayOffset)--;
		this->lcd->clearDisplay();

		displayMenu(current);

	}

	// Place the cursor at the right position on the display
	int displayPosition[4] = {0x80, 0xC0, 0x94, 0xD4};
	this->lcd->lcd_write(displayPosition[*vc - *displayOffset], 0);

	return 0;

}


