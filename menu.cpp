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


/*** Class MenuNode: ***/

// Constructors
MenuNode::MenuNode(){
}
MenuNode::MenuNode(std::string name) {
	this->name = name;
	this->pressure = NULL;
	displayOffset = 0;
	displayOffsetPtr = &displayOffset;
	vectorCursor = 0;

}
/*
MenuNode::MenuNode(std::string name, MenuNode *parent) {
	this->name = name;
	this->v.push_back(parent);
}
*/
//Member Functions
void MenuNode::setChild(MenuNode *child) {
	child->v.push_back(this);
	this->v.push_back(child);
}
void MenuNode::moveUp(bool up) {
	// if(up)
}
void MenuNode::moveDown(bool down) {
	// if(down)
}
void MenuNode::action(bool select) {
	// if(select)
}
std::string MenuNode::getName() {
	return this->name;
}
void MenuNode::setName(std::string name) {
	this->name = name;
}
std::vector<MenuNode*> MenuNode::getVector() {
	return this->v;
}
void MenuNode::setVector(std::vector<MenuNode*> v) {
	this->v = v;
}

// Destructor
MenuNode::~MenuNode() {
}

void MenuNode::setPressure(int pres) {
	 this->pressure = pres;
}
int MenuNode::getPressure() {
	 return this->pressure;
}
void MenuNode::setForce(int force) {
	 this->force = force;
}
int MenuNode::getForce() {
	 return this->force;
}


/*** Class MenuController: ***/
// Constructor
MenuController::MenuController() {
	this->current = NULL;
}

// Member functions
void MenuController::setCurrent(MenuNode *current) {
	this->current = current;
}
MenuNode* MenuController::getCurrent() {
	 return this->current;
}

// Destructor
MenuController::~MenuController() {
}

/*** Class MenuTerminalNode ***/
/*
// Constructor
MenuTerminalNode::MenuTerminalNode() {
	this->value = 0;
}

// Member functions
void MenuTerminalNode::setValue(int val) {
	 this->value = val;
}
int MenuTerminalNode::getValue() {
	 return this->value;
}
*/

/*** Class MenuNodeFactory ***/
MenuNodeFactory::MenuNodeFactory() {
}
MenuNode* MenuNodeFactory::createMenuNode(std::string name) {
	return new MenuNode(name);
}
