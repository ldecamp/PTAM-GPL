#ifndef __BRISK_L_H
#define __BRISK_L_H

#include <cvd/image.h>

class BriskLayer{
public:
	struct LayerType{
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
	inline uint8_t getAgastScore(int x, int y, uint8_t threshold);
	inline uint8_t getAgastScore_5_8(int x, int y, uint8_t threshold);
	inline uint8_t getAgastScore(float xf, float yf, uint8_t threshold, float scale=1.0f);

	// accessors
	inline const CVD::Image<CVD::byte>& img() const {return img_;}
	inline const CVD::Image<CVD::byte>& scores() const {return scores_;}
	inline float scale() const {return scale_;}
	inline float offset() const {return offset_;}

	// half sampling
	static inline void halfsample(const CVD::Image<CVD::byte> srcimg_, CVD::Image<CVD::byte> dstimg);
	// two third sampling
	static inline void twothirdsample(const CVD::Image<CVD::byte> srcimg_, CVD::Image<CVD::byte> dstimg);
private:
	
	// access gray values (smoothed/interpolated)
	__inline__ uint8_t value(const CVD::Image<CVD::byte> img_, float xf, float yf, float scale);
	
	CVD::Image<CVD::byte> img_;                // The pyramid level pixels
	CVD::Image<CVD::byte> scores_;             // The fast scores
	float scale_; // coordinate transformation
	float offset_; 
};
#endif