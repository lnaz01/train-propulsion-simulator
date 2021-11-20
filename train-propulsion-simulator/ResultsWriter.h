//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RESULTS_WRITER_DEF
#define RESULTS_WRITER_DEF

#include <fstream>
#include <string>
#include <vector>

class Point;

class ResultsWriter {

public:

	// fp		-->	File path
	// appnd	-->	Append boolean
	ResultsWriter(std::string fp, bool appnd);

	virtual ~ResultsWriter();

	// File path
	std::string fp;

	// Output file stream
	std::ofstream* ofs;

	// Write data vector to single line
	template <typename T>
	void writeLine(std::vector<T> d);

	// Writes points array to multiple lines (one point per line)
	// points		-->	Array of points
	// pointsSize	--> Size of array of points
	void writePoints(Point* points, int pointsSize);

};

// Writes data vector to single line
// d	-->	Data
template <typename T>
void ResultsWriter::writeLine(std::vector<T> d) {
	for (size_t i = 0; i < d.size(); i++) {
		*ofs << d[i];
		if (i == (d.size() - 1)) {
			*ofs << std::endl;
		}
		else {
			*ofs << ",";
		}
	}
}

#endif
