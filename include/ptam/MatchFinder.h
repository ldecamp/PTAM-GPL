// Goal of this class is to provide helper that find correspondance between 2 points in successive frames
#ifndef __MATCHFINDER_H
#define __MATCHFINDER_H

#include "KeyFrame.h"


class MatchFinder
{
public:
	MatchFinder(int maxDistance);

	bool FindMatchCoarse(const KeyFrame& kf, const Feature& ft, Feature& ft2, unsigned int range);
protected:
	int mMaxDistance;
};


#endif