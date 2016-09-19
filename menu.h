/*
 * menu.h
 *
 *  Created on: 2 May 2016
 *      Author: mkadrummond
 */

#ifndef MENU_H_
#define MENU_H_

/* Menu for a 4 line HD44780 display
 * MenuController
 * MenuEllement
 */

#include <vector>
#include <string>
#include "eqep.h"

class MenuNode;

class MenuNode {
public:
	MenuNode();
	MenuNode(std::string);	// constructor with name
	MenuNode(std::string, MenuNode*);	// constructor with parent

	std::string name;
	std::vector<MenuNode*> v;	// vector that holds the children
									// the first element may also be the parent
	int displayOffset;
	int *displayOffsetPtr;
	int vectorCursor;


	void setChild(MenuNode*);
	void moveUp(bool);
	void moveDown(bool);
	void action(bool);
	std::string getName();
	void setName(std::string);

	std::vector<MenuNode*> getVector();
	void setVector(std::vector<MenuNode*>);
	//void useElement();
	//int passPtr(int (*pt2Func)(int));
	//int useElement (int x, eQEP& encoder, int (*moo)(int, eQEP&));
	int useElement (int x, int (*moo)(int));

	void setPressure(int);
	int getPressure();
	void setForce(int);
	int getForce();

	~MenuNode();

private:
	int pressure;
	int length;
	int force;

};

class MenuController {
public:
	MenuController();
	MenuNode *current;

	MenuNode* getCurrent();
	void setCurrent(MenuNode*);

	~MenuController();
};

/*
class MenuTerminalNode: public MenuNode {
public:
	MenuTerminalNode();
	void setValue(int);
	int getValue();
private:
	int value;
};
*/

class MenuNodeFactory {
public:
	MenuNodeFactory();
	MenuNode *createMenuNode(std::string);
};



#endif /* MENU_H_ */
