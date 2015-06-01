// Copyright 2008 Isis Innovation Limited
#include "ptam/KeyFrame.h"
#include "ptam/ShiTomasi.h"
#include "ptam/SmallBlurryImage.h"
#include <cvd/vision.h>
#include <cvd/fast_corner.h>
#include <brisk/brisk.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

//TODO: Change keyframe structure to cope with new descriptors

void KeyFrame::MakeKeyFrame_Lite(Image<byte> &im)
{
  // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  
  //Get Brisk Corners
  std::vector<KeyPoint> keypoints;
  BriskScaleSpace briskScaleSpace(2);
  briskScaleSpace.constructPyramid(im);
  briskScaleSpace.getKeypoints(10,keypoints);

  // First, copy out the image data to the pyramid's zero level.
  aLevels[0].im.resize(im.size());
  copy(im, aLevels[0].im);

  // Then, for each level...
  //Initialise object
  for(int i=0; i<LEVELS; i++)
  {
    Level &lev = aLevels[i];
    if(i!=0)
	  {  //Get From brisk Layer
        lev.im = briskScaleSpace.getPyramid()[i].img();
	  }
    lev.vCorners.clear();
    lev.vCornerRowLUT.clear();
  }

  //fill corners
  for(unsigned int i=0;i<keypoints.size();i++){
    // if(keypoints[i].octave==0)
      aLevels[keypoints[i].octave].vCorners.push_back(CVD::ImageRef((int)keypoints[i].pt.x,(int)keypoints[i].pt.y));
  }

  //add index
  for(int i=0; i<LEVELS; i++)
  {
    Level &lev = aLevels[i];
    // Generate row look-up-table for the FAST corner points: this speeds up 
    // finding close-by corner points later on.
    unsigned int v=0;
    for(int y=0; y<=lev.im.size().y; y++)
    {
      while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
        v++;
      lev.vCornerRowLUT.push_back(v);
    }
  }
}

void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  
  // For each level...
  for(int l=0; l<LEVELS; l++)
    {
      Level &lev = aLevels[l];
      // .. find those FAST corners which are maximal..
      fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
      // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
      // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
      // to make new map points out of.
      for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
	{
	  if(!lev.im.in_image_with_border(*i, 10))
	    continue;
	  double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
	  if(dSTScore > *gvdCandidateMinSTScore)
	    {
	      Candidate c;
	      c.irLevelPos = *i;
	      c.dSTScore = dSTScore;
	      lev.vCandidates.push_back(c);
	    }
	}
    };
  
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);  
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();
}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
Level& Level::operator=(const Level &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.resize(rhs.im.size());
  copy(rhs.im, im);
  
  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
      {
	if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0); //Red
	else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0); //Yellow
	else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0); // Green
	else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 0.7); // Blue
	else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
      }
  }
};
static LevelHelpersFiller foo;







