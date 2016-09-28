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

namespace Menu {

	class MenuNode;

	class MenuNode {
	public:

		float pressure;
		int length;
		float force;

		MenuNode();
		MenuNode(std::string);	// constructor with name
		MenuNode(std::string, MenuNode*);	// constructor with parent
		//MenuNode(std::string, MenuNode*, float, int);	// with pressure and length
		MenuNode(std::string, int (*)(void));	// constructor with useNode function
		MenuNode(std::string, MenuNode*, int (*)(void));	// constructor with parent and useNode function

		std::string name;
		std::vector<MenuNode*> v;	// vector that holds the children
										// the first element may also be the parent
		int displayOffset;
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
		int passPtr(int (*pt2Func)());
		//int useElement (int (*moo)(int, eQEP&));
		int (*useNode)();


		/*
		void setPressure(float);
		float getPressure();
		void setLength(int);
		int getLength();
		void setForce(float);
		float getForce();
		*/

		~MenuNode();

	private:


	};
/*
	class MenuController {
	public:
		MenuController();
		MenuNode *current;

		MenuNode* getCurrent();
		void setCurrent(MenuNode*);

		~MenuController();
	};
*/
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
		MenuNode *createMenuNode(std::string, int (*)(void));
		MenuNode *createMenuNode(std::string, int (*)(void), float, int);
	};

	// A variable to keep track of where we are in the menu
	extern Menu::MenuNode *current;

}

#endif /* MENU_H_ */
