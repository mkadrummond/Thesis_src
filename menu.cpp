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
Menu::MenuNode::MenuNode(std::string n)
	:	name(n),
		pressure(0),
		force(0),
		length(0),
		displayOffset(0),
		vectorCursor(0)
		{
		}

Menu::MenuNode::MenuNode(std::string n, MenuNode *parent)
	:	name(n),
		pressure(0),
		force(0),
		length(0),
		displayOffset(0),
		vectorCursor(0)
		{
	parent->setChild(this);

		}

Menu::MenuNode::MenuNode(std::string n, int (*pt2Func)())
	:	name(n),
		pressure(0),
		force(0),
		length(0),
		displayOffset(0),
		vectorCursor(0),
		useNode(pt2Func)
		{
		}

Menu::MenuNode::MenuNode(std::string n, MenuNode *parent, int (*pt2Func)())
	:	name(n),
		pressure(0),
		force(0),
		length(0),
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
void Menu::MenuNode::moveUp(bool up) {
	// if(up)
}
void Menu::MenuNode::moveDown(bool down) {
	// if(down)
}
void Menu::MenuNode::action(bool select) {
	// if(select)
}
std::string Menu::MenuNode::getName() {
	return this->name;
}
void Menu::MenuNode::setName(std::string name) {
	this->name = name;
}
std::vector<Menu::MenuNode*> Menu::MenuNode::getVector() {
	return this->v;
}
void Menu::MenuNode::setVector(std::vector<MenuNode*> v) {
	this->v = v;
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
/*
// Private member functions:
void Menu::MenuNode::setPressure(float pres) {
	 this->pressure = pres;
}
float Menu::MenuNode::getPressure() {
	 return this->pressure;
}
void Menu::MenuNode::setLength(int length) {
	 this->length = length;
}
int Menu::MenuNode::getLength() {
	 return this->length;
}
void Menu::MenuNode::setForce(float force) {
	 this->force = force;
}
float Menu::MenuNode::getForce() {
	 return this->force;
}
*/

/*** Class MenuNodeFactory ***/
Menu::MenuNodeFactory::MenuNodeFactory() {
}
Menu::MenuNode* Menu::MenuNodeFactory::createMenuNode(std::string name) {
	return new MenuNode(name);
}
Menu::MenuNode* Menu::MenuNodeFactory::createMenuNode(std::string name, int (*pt2Func)()) {
	return new MenuNode(name, pt2Func);
}
//Menu::MenuNode* Menu::MenuNodeFactory::createMenuNode(std::string name, int (*pt2Func)(), float pressure, int length) {
//	return new MenuNode(name, pt2Func, pressure, length);
//}
