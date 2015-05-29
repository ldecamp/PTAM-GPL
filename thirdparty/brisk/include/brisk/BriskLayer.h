#ifndef __BRISK_L_H
#define __BRISK_L_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>
#include <stdint.h>
#include <agast/cvWrapper.h>
#include <agast/agast5_8.h>
#include <agast/oast9_16.h>

namespace CVD{

class BriskLayer{
public:
	struct LayerTypes{
		static const int HALFSAMPLE=0;
		static const int TWOTHIRDSAMPLE=1;
	};

	// construct a base layer
	BriskLayer(const CVD::Image<CVD::byte>& img, float scale=1.0f, float offset=0.0f);
	// derive a layer
	BriskLayer(const BriskLayer& layer, int mode);

	// Fast/Agast without non-max suppression
	void getAgastPoints(uint8_t threshold, std::vector<CVD::ImageRef>& keypoints);

	// get scores - attention, this is in layer coordinates, not scale=1 coordinates!
	uint8_t getAgastScore(int x, int y, uint8_t threshold);
	uint8_t getAgastScore_5_8(int x, int y, uint8_t threshold);
	uint8_t getAgastScore(float xf, float yf, uint8_t threshold, float scale=1.0f);

	// accessors
	inline const CVD::Image<CVD::byte>& img() const {return img_;}
	inline const CVD::Image<CVD::byte>& scores() const {return scores_;}
	inline float scale() const {return scale_;}
	inline float offset() const {return offset_;}

private:
	
	// access gray values (smoothed/interpolated)
	__inline__  uint8_t value(const CVD::Image<unsigned char>& mat, float xf, float yf, float scale);
	
	CVD::Image<CVD::byte> img_;                // The pyramid level pixels
	CVD::Image<unsigned char> scores_;             // The fast scores
	float scale_; // coordinate transformation
	float offset_;

	agast::OastDetector9_16* oastDetector_;
	agast::AgastDetector5_8* agastDetector_5_8_; 
};
}
#endif
