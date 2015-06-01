#ifndef __BRISK_FEX_H
#define __BRISK_FEX_H

#include "KeyPoint.hpp"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>
#include <stdint.h>
#include <emmintrin.h>
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
}

#include "hammingsse.hpp"

#endif