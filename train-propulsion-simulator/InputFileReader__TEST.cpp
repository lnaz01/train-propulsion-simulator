//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include <iostream>
#include "InputFileReader.h"


class MyInputFileReader : public InputFileReader {

public:

	MyInputFileReader(std::string absoluteFilePath);

	virtual std::string load() override;

};


MyInputFileReader::MyInputFileReader(std::string absoluteFilePath) :
	InputFileReader(absoluteFilePath) {}


std::string MyInputFileReader::load() {
	return "";
}


void inputFileExistsBool__TEST(MyInputFileReader& myInputFileReader);
void inputFileAsbolutePath__TEST(MyInputFileReader& myInputFileReader);
void inputFileDirectoryPath__TEST(MyInputFileReader& myInputFileReader);
void inputFileName__TEST(MyInputFileReader& myInputFileReader);
void currentLine__TEST(MyInputFileReader& myInputFileReader);
void currentLineNumber__TEST(MyInputFileReader& myInputFileReader);
void totalNumberOfLines__TEST(MyInputFileReader& myInputFileReader);


int main() {

	std::string absoluteFilePath = "C:/Users/Leith/Desktop/my_test_file.txt";
	MyInputFileReader myInputFileReader(absoluteFilePath);
	inputFileExistsBool__TEST(myInputFileReader);
	inputFileAsbolutePath__TEST(myInputFileReader);
	inputFileDirectoryPath__TEST(myInputFileReader);
	inputFileName__TEST(myInputFileReader);
	myInputFileReader.reinitialize();
	currentLine__TEST(myInputFileReader);
	myInputFileReader.reinitialize();
	currentLineNumber__TEST(myInputFileReader);
	myInputFileReader.reinitialize();
	totalNumberOfLines__TEST(myInputFileReader);

}


void inputFileExistsBool__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "inputFileExistsBool__TEST" << std::endl;
	std::cout << myInputFileReader.inputFileExistsBool << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}


void inputFileAsbolutePath__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "inputFileAsbolutePath__TEST" << std::endl;
	std::cout << myInputFileReader.inputFileAbsolutePath << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}


void inputFileDirectoryPath__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "inputFileDirectoryPath__TEST" << std::endl;
	std::cout << myInputFileReader.inputFileDirectoryPath << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}


void inputFileName__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "inputFileName__TEST" << std::endl;
	std::cout << myInputFileReader.inputFileName << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}


void currentLine__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "currentLine__TEST" << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLine << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLine << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLine << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLine << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLine << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}


void currentLineNumber__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "currentLineNumber__TEST" << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLineNumber << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLineNumber << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLineNumber << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLineNumber << std::endl;
	myInputFileReader.nextLine();
	std::cout << myInputFileReader.currentLineNumber << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}


void totalNumberOfLines__TEST(MyInputFileReader& myInputFileReader) {
	std::cout << "totalNumberOfLines__TEST" << std::endl;
	std::cout << myInputFileReader.totalNumberOfLines << std::endl;
	std::cout << std::endl << std::endl << std::endl << std::endl;
}

