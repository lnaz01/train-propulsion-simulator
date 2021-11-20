//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Point.h"
#include "Function.h"


void console__TEST();
void copy__TEST();
void load__TEST();


int main() {

	console__TEST();
	copy__TEST();
	load__TEST();

}


void console__TEST() {
	Point pt1(5.5, 20.2);
	std::cout << "console__TEST" << std::endl;
	std::cout << "pt1: " << std::endl;
	pt1.console();
	std::cout << std::endl << std::endl;
}


void copy__TEST() {
	Point pt1(5.5, 20.2);
	Point pt1_copy;
	pt1_copy.copy(&pt1);
	std::cout << "copy__TEST" << std::endl;
	std::cout << "pt1: " << std::endl;
	pt1.console();
	std::cout << std::endl;
	std::cout << "pt1_copy: " << std::endl;
	pt1_copy.console();
	std::cout << std::endl << std::endl;
}


void load__TEST() {
	Function* fnc = new Function(false, 0);
	std::string strpt = "5.9, 20.5";
	Point* point1 = new Point();
	std::string strval = (*point1).load(strpt, fnc);
	std::cout << "load__TEST" << std::endl;
	std::cout << "strval: " << strval << std::endl;
	std::cout << "point1: " << std::endl;
	point1->console();
	std::cout << std::endl << std::endl << std::endl;
}

