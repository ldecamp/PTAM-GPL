#include <math.h>

#ifndef M_PI
    #define M_PI 3.141592653589793
#endif

struct {
	float x;
	float y;

	Point2f(float x, float y) : x(x), y(y) {}

	Point2f& operator=(const Point2f &pt) {
        x=pt.x;
        y=pt.y;
        return *this;
    }

    Point2f operator+(const Point2f &pt) const{
        return Point2f(x+pt.x,y+pt.y);
    }

    Point2f operator-(const Point2f &pt) const {
      	return Point2f(x-pt.x,y-pt.y);
    }

    bool operator==(const Point2f &pt) const
    {
    	return (x==pt.x && y==pt.y);
    }

    inline float norm(){
    	return sqrt(pow(x,2)+pow(y,2));
    }

} Point2f;

struct KeyPoint{
	float angle;
	int octave;
	Point2f pt;
	float response;
	float size;

	static inline float overlap(const KeyPoint &kp1, const KeyPoint &kp2);
};
