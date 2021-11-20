//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INPUT_FILE_READER_DEF
#define INPUT_FILE_READER_DEF

#include <fstream>
#include <string>
#include <vector>

class InputFileReader {

public:

	InputFileReader(std::string inputFileAbsolutePath);

	InputFileReader();

	virtual ~InputFileReader();

	// Comment character
	static constexpr char COMMENT_CHARACTER = '#';

	// Valid input string
	static const std::string VALID_INPUT_STRING;

	// Input file exists boolean
	bool inputFileExistsBool;

	// Input file absolute path
	std::string inputFileAbsolutePath;

	// Input file directory path
	std::string inputFileDirectoryPath;

	// Input file name (without extension)
	std::string inputFileName;

	// Current line
	std::string currentLine;

	// Current line number
	int currentLineNumber;

	// Total number of lines in file
	int totalNumberOfLines;

	// Splits string into vector of strings based on character 'c'
	static std::vector<std::string> split_string(std::string str, char c);

	// Loads data from file
	virtual std::string load() = 0;

	// Reads next line of file
	void nextLine();

	// Re-initialize input file reader
	void reinitialize();

protected:

	// Input file stream
	std::ifstream* inputFileStream;

	// Check for empty file
	bool fileIsEmpty();

	// Removes white space and comments from string
	void removeWhiteSpaceAndComments();

	// Calculates input file directory path
	void calc_inputFileDirectoryPath();

	// Calculates input file name
	void calc_inputFileName();

	// Calculates total number of lines in file
	void calc_totalNumberOfLines();

};

#endif
