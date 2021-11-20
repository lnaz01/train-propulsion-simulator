//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "InputFileReader_ForcedSpeed.h"
#include "InputFileReader_Simulation.h"
#include "Track.h"


class MyTrack : public Track {
public:
	MyTrack(InputFileReader_Simulation* ifrsim);
	void set_trackLength(double trackLength);
};


MyTrack::MyTrack(InputFileReader_Simulation* ifrsim) : Track(ifrsim) {}


void MyTrack::set_trackLength(double trackLength) {
	this->trackLength = trackLength;
}


int main() {

	std::string idfp = "C:/Users/Leith/Desktop/Simulation_TEST.txt";
	InputFileReader_Simulation* ifrsim = new InputFileReader_Simulation(idfp);
	ifrsim->userDefinedTracks.push_back(new MyTrack(ifrsim));
	((MyTrack*)(ifrsim->userDefinedTracks[0]))->set_trackLength(40000.0);
	InputFileReader_ForcedSpeed* ifrfs = new InputFileReader_ForcedSpeed(ifrsim);
	if (ifrfs->inputFileExistsBool == true) {
		std::string errstr = ifrfs->load();
		std::cout << "Returned String: " << errstr << std::endl;
	}
	else {
		std::cout << "Forced speed file (forced_speed.fsf) does not exist in directory." << std::endl;
	}

}

