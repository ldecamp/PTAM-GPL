#ifndef __BRISK_FDET_H
#define __BRISK_FDET_H

#include "KeyPoint.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>

namespace CVD{
	
class BriskFeatureDetector
{
public:
	BriskFeatureDetector(int threshold, int octaves=4);
	int threshold;
	int octaves;
	void detect(const CVD::Image<CVD::byte>& image, std::vector<KeyPoint>& keypoints);
};
}
#endif 