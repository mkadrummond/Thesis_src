/**
 *  @file 	 menu.h
 *  @author  Michael Drummond
 *  @date	 02/05/2016
 *  @version 1.0
 *
 *  @brief	 Create a menu
 *
 *  Create a menu for a multi-line LCD display that can be navigated by a
 *  rotary encoder and a button.
 *
 */

#ifndef MENU_H_
#define MENU_H_

#include <vector>
#include <unordered_map>
#include <string>

#include "eqep.h"
#include "GPIO.h"
#include "LCD.h"

/**
 * @class MenuNode
 *
 * MenuNodes are the building blocks of a menu. Conceptually, a menuNode
 * represents a state in which the user of the menu can either move on to
 * another node, or manipulate some other function on the device.
 */
class MenuNode {
public:
	/**
	 * @brief Constructor that sets its name and useNode() function
	 *
	 * @param name Name of the MenuNode
	 * @param pt2Func Function pointer for useNode()
	 */
	MenuNode(std::string name, int (*pt2Func)(void));

	/**
	 * @brief Constructor that sets name, parent and useNode() function
	 *
	 * @param name 		A string identifying the MenuNode
	 * @param parent 	A MenuNode pointer to the parent of this MenuNode
	 * @param pt2Func	Function pointer for useNode()
	 */
	MenuNode(std::string name, MenuNode* parent, int (*pt2Func)(void));
	~MenuNode(); 	///< Class destructor

	std::string name;		  ///< Acts as an identifier
	std::vector<MenuNode*> v; ///< Vector that holds the children of this node
	int displayOffset;		  ///< Keeps track of the display offset
	int vectorCursor;		  ///< Keeps track of the vector cursor position

	/**
	 * Adds a child MenuNode to this MenuNode
	 * @param child Pointer to a MenuNode
	 */
	void setChild(MenuNode* child);

	/**
	 * Returns the name of this MenuNode
	 * @return A string
	 */
	std::string getName();

	/**
	 * Sets the name of this MenuNode
	 * @param name A string
	 */
	void setName(std::string name);

//	int passPtr(int (*pt2Func)());

	/**
	 * Dereferences a function pointer
	 * @return An integer
	 */
	int (*useNode)();
};

/**
 * @class Menu
 *
 * The Menu class groups MenuNodes into an unordered map, keeps track of which
 * MenuNode is currently being displayed, and acts as the interface to the
 * encoder and button that make navigating the menu possible.
 */
class Menu {
public:
	Menu();	//!< Default constructor

	/**
	 * @brief Constructor that defines the encoder, button and LCD
	 *
	 * @param encoder 	Pointer to eQEP object
	 * @param button 	Pointer to GPIO object
	 * @param lcd		Pointer to LCD object
	 */
	Menu(eQEP* encoder, exploringBB::GPIO* button, LCD* lcd);

	~Menu();	///< Class destructor

	eQEP *encoder;	///< Pointer to object to control and read the encoder
	exploringBB::GPIO *button; ///< Pointer to object to read the button input
	LCD *lcd;		///< Pointer to object to control the LCD display
	std::unordered_map<std::string, MenuNode*> map; ///< Map of the MenuNodes
	MenuNode *current;	///< Keeps track of the users position in the menu

	/**
	 * Loads the MenuNodes into the Menu unordered map
	 * @return unordered map with the MenuNodes that make up the menu
	 */
	std::unordered_map<std::string, MenuNode*> initMenu();

	/**
	 * Displays the current MenuNode v vector to the LCD
	 * @param 	current The MenuNode whose v vector is to display
	 * @return 	An integer, 0 is success
	 */
	int displayMenu(MenuNode* current);

	/**
	 * Updates the display with the latest position of the cursor in the v
	 * vector. If the cursor has moved passed the bottom of the display, then
	 * the elements of the vector that are displayed are also shifted.
	 * @param 	current The MenuNode that is being displayed on the LCD
	 * @return	An integer, 0 is success
	 */
	int updateDisplay(MenuNode* current);
};

/**
 * @class MenuNodeFactory
 * @brief Creates new MenuNodes
 *
 * The MenuNodeFactory is used to create new MenuNodes with the factory method
 * at run time.
 */
class MenuNodeFactory {
public:
	/**
	 * Creates a pointer to a new MenuNode
	 * @param name 		A string identifying the MenuNode
	 * @param pt2func 	A function pointer for the MenuNode's useNode()
	 * @return			A pointer to the newly created MenuNode
	 */
	MenuNode *createMenuNode(std::string name, int (*pt2func)(void));
};

#endif /* MENU_H_ */
