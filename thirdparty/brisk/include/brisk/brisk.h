#ifndef __BRISK_H
#define __BRISK_H

#include <vector>
#include <stdint.h>
#include <emmintrin.h>

#include <cvd/image.h>
#include <cvd/byte.h>

#include <agast/cvWrapper.h>
#include <agast/agast5_8.h>
#include <agast/oast9_16.h>

#include "keyPoint.hpp"

// this is needed to avoid aliasing issues with the __m128i data type:
#ifdef __GNUC__
	typedef unsigned char __attribute__ ((__may_alias__)) UCHAR_ALIAS;
	typedef unsigned short __attribute__ ((__may_alias__)) UINT16_ALIAS;
	typedef unsigned int __attribute__ ((__may_alias__)) UINT32_ALIAS;
	typedef unsigned long int __attribute__ ((__may_alias__)) UINT64_ALIAS;
	typedef int __attribute__ ((__may_alias__)) INT32_ALIAS;
	typedef uint8_t __attribute__ ((__may_alias__)) U_INT8T_ALIAS;
#endif
#ifdef _MSC_VER
	// Todo: find the equivalent to may_alias
	#define UCHAR_ALIAS unsigned char //__declspec(noalias)
	#define UINT32_ALIAS unsigned int //__declspec(noalias)
	#define __inline__ __forceinline
#endif

namespace CVD{

// some helper structures for the Brisk pattern representation
struct BriskPatternPoint{
	float x;         // x coordinate relative to center
	float y;         // x coordinate relative to center
	float sigma;     // Gaussian smoothing sigma
};
struct BriskShortPair{
	unsigned int i;  // index of the first pattern point
	unsigned int j;  // index of other pattern point
};
struct BriskLongPair{
	unsigned int i;  // index of the first pattern point
	unsigned int j;  // index of other pattern point
	int weighted_dx; // 1024.0/dx
	int weighted_dy; // 1024.0/dy
};


class BriskDescriptorExtractor {
	public:
		// create a descriptor with standard pattern
		BriskDescriptorExtractor(bool rotationInvariant=true, bool scaleInvariant=true, float patternScale=1.0f);
		// custom setup
		BriskDescriptorExtractor(std::vector<float> &radiusList, std::vector<int> &numberList,
			bool rotationInvariant=true, bool scaleInvariant=true,
			float dMax=5.85f, float dMin=8.2f, std::vector<int> indexChange=std::vector<int>());
		virtual ~BriskDescriptorExtractor();

		// call this to generate the kernel:
		// circle of radius r (pixels), with n points;
		// short pairings with dMax, long pairings with dMin
		void generateKernel(std::vector<float> &radiusList,
			std::vector<int> &numberList, float dMax=5.85f, float dMin=8.2f,
			std::vector<int> indexChange=std::vector<int>());

		bool rotationInvariance;
		bool scaleInvariance;

		virtual void compute(const CVD::Image<CVD::byte>& image, 
			std::vector<KeyPoint>& keypoints, 
			CVD::Image<CVD::byte>& descriptors) const;

	protected:
		__inline__ int smoothedIntensity(const CVD::Image<CVD::byte>& image,
				const CVD::Image<int>& integral,const float key_x,
					const float key_y, const unsigned int scale,
					const unsigned int rot, const unsigned int point) const;
		// pattern properties
		BriskPatternPoint* patternPoints_; 	//[i][rotation][scale]
		unsigned int points_; 				// total number of collocation points
		float* scaleList_; 					// lists the scaling per scale index [scale]
		unsigned int* sizeList_; 			// lists the total pattern size per scale index [scale]
		static const unsigned int scales_;	// scales discretization
		static const float scalerange_; 	// span of sizes 40->4 Octaves - else, this needs to be adjusted...
		static const unsigned int n_rot_;	// discretization of the rotation look-up

		// pairs
		int strings_;						// number of uchars the descriptor consists of
		float dMax_; 						// short pair maximum distance
		float dMin_; 						// long pair maximum distance
		BriskShortPair* shortPairs_; 		// d<_dMax
		BriskLongPair* longPairs_; 			// d>_dMin
		unsigned int noShortPairs_; 		// number of shortParis
		unsigned int noLongPairs_; 			// number of longParis

		// general
		static const float basicSize_;
	};

	/// Faster Hamming distance functor - uses sse
	/// bit count of A exclusive XOR'ed with B
	class HammingSse
	{
	public:
		HammingSse(){};

		// SSSE3 - even faster!
		static __inline__ uint32_t ssse3_popcntofXORed(const __m128i* signature1,
				const __m128i* signature2, const int numberOf128BitWords);

	    typedef unsigned char ValueType;

	    //! important that this is signed as weird behavior happens
	    // in BruteForce if not
	    typedef int ResultType;

	    // this will count the bits in a ^ b
	    ResultType operator()(const unsigned char* a, const unsigned char* b, const int size) const
	    {
		return ssse3_popcntofXORed(
				(const __m128i*)(a),
				(const __m128i*)(b),
				size/16);
	    }
	};


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

class BriskFeatureDetector
{
public:
	BriskFeatureDetector(int threshold, int octaves=4);
	int threshold;
	int octaves;
	void detect(const CVD::Image<CVD::byte>& image, std::vector<KeyPoint>& keypoints);
};

}

#include "hammingsse.hpp"

#endif