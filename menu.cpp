/*
 * menu.cpp
 *
 *  Created on: 2 May 2016
 *      Author: mkadrummond
 */

#include "menu.h"
#include "lcdDisplay.h"
#include <iostream>
using namespace std;

namespace Menu {
	Menu::MenuNode *current;
}

/*** Class MenuNode: ***/

Menu::MenuNode::MenuNode(std::string n, int (*pt2Func)())
	:	name(n),
		displayOffset(0),
		vectorCursor(0),
		useNode(pt2Func)
		{
		}

Menu::MenuNode::MenuNode (std::string n, MenuNode *parent, int (*pt2Func)())
	:	name(n),
		displayOffset(0),
		vectorCursor(0),
		useNode(pt2Func)
		{
			parent->setChild(this);
		}

//Member Functions
void Menu::MenuNode::setChild(MenuNode *child) {
	child->v.push_back(this);
	this->v.push_back(child);
}

std::string Menu::MenuNode::getName() {
	return this->name;
}

void Menu::MenuNode::setName(std::string name) {
	this->name = name;
}

int Menu::MenuNode::passPtr(int (*pt2Func)()) {
	return pt2Func();
}
int Menu::MenuNode::*useNode() {
	return 0;
}

// Destructor
Menu::MenuNode::~MenuNode() {
}


/*** Class MenuNodeFactory ***/
Menu::MenuNode* Menu::MenuNodeFactory::createMenuNode(std::string name, int (*pt2Func)()) {
	return new MenuNode(name, pt2Func);
}

