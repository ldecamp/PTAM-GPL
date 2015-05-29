#include "KeyPoint.h"

float KeyPoint::overlap( const KeyPoint& kp1, const KeyPoint& kp2 )
{
    float a = kp1.size * 0.5f;
    float b = kp2.size * 0.5f;
    float a_2 = a * a;
    float b_2 = b * b;

    Point2f p1 = kp1.pt;
    Point2f p2 = kp2.pt;
    float c = (float)(p1 - p2).norm();

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