#ifndef __BRISK_FDET_H
#define __BRISK_FDET_H

#include "KeyPoint"

class BriskFeatureDetector
{
public:
	BriskFeatureDetector(int thresh, int octaves=4);
	int threshold;
	int octaves;
	void detect(const CVD::Image<CVD::byte>& img, std::vector<KeyPoint>& keypoints);
};

#endif 