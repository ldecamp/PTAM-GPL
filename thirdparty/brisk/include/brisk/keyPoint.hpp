#ifndef __CVD_KEYPOINT_H
#define __CVD_KEYPOINT_H

#include <math.h>
#include <cvd/image.h>

#ifndef M_PI
    #define M_PI 3.141592653589793
#endif

namespace CVD{

	struct Point2f{
		float x;
		float y;

		Point2f(){}
		Point2f(float x, float y): x(x), y(y){}

		Point2f& operator=(const Point2f& pt){
			x=pt.x;
			y=pt.y;
			return *this;
		}

		Point2f operator+(const Point2f& pt) const{
			return Point2f(pt.x+x, pt.y+y);
		}

		Point2f operator-(const Point2f& pt) const{
			return Point2f(pt.x-x, pt.y-y);
		}

		bool operator==(const Point2f& pt) const{
			return (pt.x==x&&pt.y==y);
		}

		inline float mag_squared(){
			return sqrt(pow(x,2)+pow(y,2));
		}

		inline CVD::ImageRef ir(){
			return ImageRef(x,y);
		}
	};

	struct KeyPoint{
		float angle;
		int octave;
		Point2f pt;
		float response;
		float size;

		KeyPoint(){}
		KeyPoint(float _x, float _y, float _size, float _angle, float _response, float _octave)
		: angle(_angle), octave(_octave), pt(Point2f(_x,_y)), response(_response), size(_size)  {}

		KeyPoint& operator=(const KeyPoint &rhv){
			angle=rhv.angle;
			octave=rhv.octave;
			pt=Point2f(rhv.pt.x,rhv.pt.y);
			response=rhv.response;
			size=rhv.size;
			return *this;
		}

		static inline float overlap( const KeyPoint& kp1, const KeyPoint& kp2 )
		{
			float a = kp1.size * 0.5f;
			float b = kp2.size * 0.5f;
			float a_2 = a * a;
			float b_2 = b * b;

			Point2f p1 = kp1.pt;
			Point2f p2 = kp2.pt;
			float c = (float)(p1 - p2).mag_squared();

			float ovrl = 0.f;

    // one circle is completely encovered by the other => no intersection points!
			if( std::min( a, b ) + c <= std::max( a, b ) )
				return std::min( a_2, b_2 ) / std::max( a_2, b_2 );

    if( c < a + b ) // circles intersect
    {
    	float c_2 = c * c;
    	float cosAlpha = ( b_2 + c_2 - a_2 ) / ( kp2.size * c );
    	float cosBeta  = ( a_2 + c_2 - b_2 ) / ( kp1.size * c );
    	float alpha = acos( cosAlpha );
    	float beta = acos( cosBeta );
    	float sinAlpha = sin(alpha);
    	float sinBeta  = sin(beta);

    	float segmentAreaA = a_2 * beta;
    	float segmentAreaB = b_2 * alpha;

    	float triangleAreaA = a_2 * sinBeta * cosBeta;
    	float triangleAreaB = b_2 * sinAlpha * cosAlpha;

    	float intersectionArea = segmentAreaA + segmentAreaB - triangleAreaA - triangleAreaB;
    	float unionArea = (a_2 + b_2) * (float)M_PI - intersectionArea;

    	ovrl = intersectionArea / unionArea;
    }

    return ovrl;
}
};

}
#endif
