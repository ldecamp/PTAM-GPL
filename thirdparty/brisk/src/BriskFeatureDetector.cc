#include "BriskFeatureDetector.h"
#include "BriskScaleSpace.h"

#include <cvd/vision.h>

using namespace std;
using namespace CVD;

BriskFeatureDetector::BriskFeatureDetector(int threshold, int octaves){
	threshold=threshold;
	octaves=octaves;
}

void BriskFeatureDetector::detect(const CVD::Image<CVD::byte>& image, std::vector<KeyPoint>& keypoints){
	BriskScaleSpace briskScaleSpace(octaves);
	briskScaleSpace.constructPyramid(image);
	briskScaleSpace.getKeypoints(threshold,keypoints);
	//TODO: need to remove invalid points.
}