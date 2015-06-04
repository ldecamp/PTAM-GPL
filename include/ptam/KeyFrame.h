// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <brisk/keyPoint.hpp>
#include <vector>
#include <set>
#include <map>

class MapPoint;
class SmallBlurryImage;

//Needs to be odds
#define LEVELS 4

//Defines a structure for Storing Corner location + descriptor information
struct Feature{
  CVD::Point2f ptRootPos;
  unsigned char* descriptor;
  int octave;

  inline Feature(){
    descriptor=new unsigned char[64];
    octave=0;
  }
  inline Feature(CVD::Point2f ptPos, unsigned char descriptor_[64]){
    ptRootPos=ptPos;
    descriptor=descriptor_;
    octave=0;
  }
  inline Feature(CVD::Point2f ptPos, unsigned char descriptor_[64], int octave_){
    ptRootPos=ptPos;
    descriptor=descriptor_;
    octave=octave_;
  }
  inline Feature(const CVD::KeyPoint& keypoint, unsigned char descriptor_[64]) {
    ptRootPos=keypoint.pt;
    descriptor=descriptor_;
    octave=keypoint.octave;
  }

  Feature& operator=(const Feature& ft){
    ptRootPos=ft.ptRootPos;
    descriptor=ft.descriptor;
    octave=ft.octave;
    return *this;
  }

};

// Candidate: a feature in an image which could be made into a map point
struct Candidate : public Feature{
  double dSTScore;
  inline Candidate(){}
  inline Candidate(const Feature& feature, double sTSscore)
    :dSTScore(sTSscore) {
    ptRootPos=feature.ptRootPos;
    descriptor=feature.descriptor;
    octave=feature.octave;
  }

  bool operator<(const Candidate& c)const{
    return (dSTScore<c.dSTScore&&ptRootPos<c.ptRootPos);
  }

  bool operator==(const Candidate& c) const{
    return (dSTScore==c.dSTScore&&ptRootPos==c.ptRootPos);
  }
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
struct Measurement
{
  int nLevel;   // Which image level?
  bool bSubPix; // Has this measurement been refined to sub-pixel level?
  Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

struct ScaleSpace{
  CVD::Image<CVD::byte> im; // store the pyramid level pixels  
  float scale;
  float offset; 

  inline ScaleSpace(){
    bImplaneCornersCached=false;
  }

  std::vector<Feature> vFeatures;   //stores information about brisk features
  std::vector<int> vFeaturesLUT; //Row-index into features, for speed up access
  std::vector<Candidate> vCandidates;   // Potential locations of new map points

  ScaleSpace& operator=(const ScaleSpace &rhs);

  //Perf Optimisation
  bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
  std::vector<Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these 
// are then simply discarded, but sometimes they're then just added to the map.
struct KeyFrame
{
  inline KeyFrame()
  {
    pSBI = NULL;
  }
  SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation
  bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)
  
  std::map<MapPoint*, Measurement> mMeasurements;           // All the measurements associated with the keyframe
  
  ScaleSpace pyramid[LEVELS]; //Representation of Pyramid scale space + features extracted
  std::vector<Feature> vFeatures;   //stores information about brisk features from all layers
  std::vector<int> vFeaturesLUT; //Row-index into features, for speed up access overall indexer

  void MakeKeyFrame_Lite(CVD::Image<CVD::byte> &im);   // This takes an image and calculates pyramid levels etc to fill the 
                                                            // keyframe data structures with everything that's needed by the tracker..
  void MakeKeyFrame_Rest();                                 // ... while this calculates the rest of the data which the mapmaker needs.
  
  double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.
  double dSceneDepthSigma;
  
  SmallBlurryImage *pSBI; // The relocaliser uses this
  int mbKFId; // Save the index of the frame in video (needed for stats)
};

typedef std::map<MapPoint*, Measurement>::iterator meas_it;  // For convenience, and to work around an emacs paren-matching bug

#endif

