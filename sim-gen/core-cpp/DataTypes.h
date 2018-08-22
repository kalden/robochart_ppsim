#ifndef ROBOCALC_DATATYPES_H_
#define ROBOCALC_DATATYPES_H_

#include <string>

class Double2D {
	public:
		double x;
		double y;
};
class Int2D {
	public:
		int x;
		int y;
};
class TreeMap {
	public:
		double chemokineLevel;
		int gridSquare;
};
class Tracking_Stats {
	public:
		double timeTracked;
		double trackLength;
		double trackLengthScaled;
		double trackedVelocity;
		Double2D agentTrackStartLocation;
		Double2D agentTrackEndLocation;
		Double2D agentPreviousLocation;
};

#endif
