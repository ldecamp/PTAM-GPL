#ifndef __BRISK_SS_H
#define __BRISK_SS_H

#include "KeyPoint.h"
#include "BriskLayer.h"

class BriskScaleSpace
	{
	public:
		// construct telling the octaves number:
		BriskScaleSpace(uint8_t _octaves=4);
		~BriskScaleSpace();

		// construct the image pyramids
		void constructPyramid(const CVD::Image<CVD::byte>& image);

		// get Keypoints
		void getKeypoints(const uint8_t _threshold, std::vector<KeyPoint>& keypoints);

	protected:
		// nonmax suppression:
		__inline__ bool isMax2D(const uint8_t layer,
				const int x_layer, const int y_layer);
		// 1D (scale axis) refinement:
		__inline__ float refine1D(const float s_05,
				const float s0, const float s05, float& max); // around octave
		__inline__ float refine1D_1(const float s_05,
				const float s0, const float s05, float& max); // around intra
		__inline__ float refine1D_2(const float s_05,
				const float s0, const float s05, float& max); // around octave 0 only
		// 2D maximum refinement:
		__inline__ float subpixel2D(const int s_0_0, const int s_0_1, const int s_0_2,
				const int s_1_0, const int s_1_1, const int s_1_2,
				const int s_2_0, const int s_2_1, const int s_2_2,
				float& delta_x, float& delta_y);
		// 3D maximum refinement centered around (x_layer,y_layer)
		__inline__ float refine3D(const uint8_t layer,
				const int x_layer, const int y_layer,
				float& x, float& y, float& scale, bool& ismax);

		// interpolated score access with recalculation when needed:
		__inline__ int getScoreAbove(const uint8_t layer,
				const int x_layer, const int y_layer);
		__inline__ int getScoreBelow(const uint8_t layer,
				const int x_layer, const int y_layer);

		// return the maximum of score patches above or below
		__inline__ float getScoreMaxAbove(const uint8_t layer,
				const int x_layer, const int y_layer,
				const int threshold, bool& ismax,
				float& dx, float& dy);
		__inline__ float getScoreMaxBelow(const uint8_t layer,
				const int x_layer, const int y_layer,
				const int threshold, bool& ismax,
				float& dx, float& dy);

		// the image pyramids:
		uint8_t layers_;
		std::vector<BriskLayer> pyramid_;

		// Agast:
		uint8_t threshold_;
		uint8_t safeThreshold_;

		// some constant parameters:
		static const float safetyFactor_;
		static const float basicSize_;
	};

#endif