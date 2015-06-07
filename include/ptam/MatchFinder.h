// Goal of this class is to provide helper that find correspondance between 2 points in successive frames
#ifndef __MATCHFINDER_H
#define __MATCHFINDER_H

#include "KeyFrame.h"


class MatchFinder
{
public:
	MatchFinder(int maxDistance);

	// bool FindMatchCoarse(KeyFrame& kf, const Feature& ft, std::vector<Feature>* matches, unsigned int range);
	bool FindMatchCoarse(KeyFrame& kf, const Feature& ft, Feature*& match, unsigned int range);
protected:
	int mMaxDistance;
};


#endif