/*
 * menu.h
 *
 *  Created on: 2 May 2016
 *      Author: mkadrummond
 */

#ifndef MENU_H_
#define MENU_H_

/* Menu for a 4 line HD44780 display

 */

#include <vector>
#include <string>
#include "eqep.h"

namespace Menu {

	class MenuNode;

	class MenuNode {
	public:

		MenuNode(std::string, int (*)(void));				// constructor with useNode function
		MenuNode(std::string, MenuNode*, int (*)(void));	// constructor with parent and useNode function

		// Member variables
		std::string name;
		std::vector<MenuNode*> v;		// vector that holds the children
		int displayOffset;
		int vectorCursor;

		// Member functions
		void setChild(MenuNode*);
		std::string getName();
		void setName(std::string);
		int passPtr(int (*pt2Func)());
		int (*useNode)();

		~MenuNode();

	private:


	};

	class MenuNodeFactory {
	public:
		MenuNode *createMenuNode(std::string, int (*)(void));
	};

}

#endif /* MENU_H_ */
