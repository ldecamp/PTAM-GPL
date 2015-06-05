// Copyright 2008 Isis Innovation Limited
#include "ptam/KeyFrame.h"
#include "ptam/ShiTomasi.h"
#include "ptam/SmallBlurryImage.h"
#include <cvd/vision.h>
#include <algorithm>
#include <cvd/fast_corner.h>
#include <brisk/brisk.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

const bool sortFeatures (const Feature& f1, const Feature& f2){ 
  return (f1.ptRootPos.y()<f2.ptRootPos.y()); 
}

void KeyFrame::MakeKeyFrame_Lite(Image<byte> &im)
{
  // Perpares a Keyframe from an image. Generates pyramid levels + brisk corner detection
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  static BriskScaleSpace briskScaleSpace(LEVELS/2);
  static CVD::BriskDescriptorExtractor extractor(true,true,1.0f);
  static std::vector<KeyPoint> keypoints;
  
  //Defines the threshold for the fast corner detection of BRISK
  static gvar3<int> gvdCornerThreshold("KF.BriskFastCornerThreshold", 15, SILENT);   

  keypoints.clear();
  //build the brisk scale space of the image
  briskScaleSpace.constructPyramid(im);
  briskScaleSpace.getKeypoints(*gvdCornerThreshold,keypoints);

  vFeatures.clear();
  vFeaturesLUT.clear();
  //clear existing data+copy image
  for(int i=0; i<LEVELS; i++){
      ScaleSpace &space = pyramid[i];
      space.im.resize(briskScaleSpace.getPyramid()[i].img().size());
      copy(briskScaleSpace.getPyramid()[i].img(), space.im);
      space.vFeatures.clear();
      space.vFeaturesLUT.clear();
  }

  extractor.setImage(im);
  //map keypoints according to layer
  for(unsigned int i=0; i<keypoints.size();i++){
      ScaleSpace &space = pyramid[keypoints[i].octave];
      unsigned char descriptor[64] = {};
      //try to get descriptor as well and cache it (used for tracking and mapping)
      //reject point if cannot build descriptor
      if(extractor.compute(keypoints[i], descriptor)){
        Feature f;
        f.ptRootPos=keypoints[i].pt;
        f.descriptor=descriptor;
        f.octave=keypoints[i].octave;
        space.vFeatures.push_back(f);
        vFeatures.push_back(f);
      }
  }
  
  //rebuild global index 
  std::sort(vFeatures.begin(), vFeatures.end(), sortFeatures);

  unsigned int v=0;
  for(int y=0; y<=im.size().y;y++){
    while(v < vFeatures.size() && y > vFeatures[v].ptRootPos.y()){
      v++;
    }    
    vFeaturesLUT.push_back(v);
  }
  
  //rebuild index 
  // Generate row look-up-table for the FAST corner points: this speeds up 
  // finding close-by corner points later on.
  for(int i=0; i<LEVELS; i++){
      ScaleSpace &space = pyramid[i];

      //sort list then build index
      std::sort(space.vFeatures.begin(), space.vFeatures.end(), sortFeatures);

      unsigned int v=0;
      for(int y=0; y<=space.im.size().y;y++){
        while(v < space.vFeatures.size() && y > space.vFeatures[v].ptRootPos.y())
          v++;
        space.vFeaturesLUT.push_back(v);
      }
  }
}

void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  static gvar3<double> gvdCandidateMinSTScore("KF.CandidateMinShiTomasiScore", 70, SILENT);

  // For each level...
  for(int l=0; l<LEVELS;l++){
    ScaleSpace &space = pyramid[l];
    space.vCandidates.clear();
    // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
    // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
    // to make new map points out of.
    for(unsigned int i=0;i<space.vFeatures.size();i++){
      Feature& f=space.vFeatures[i];
      //Need to get coordinate at scale for each level
      ImageRef imRef= f.ptRootPos.ir();
      if(!pyramid[0].im.in_image_with_border(imRef, 4))
        continue;
      double dSTScore = FindShiTomasiScoreAtPoint(pyramid[0].im, 3, imRef);
      
      if(dSTScore > *gvdCandidateMinSTScore)
      {
        Candidate c;
        c.ftInd=i;
        c.dSTScore=dSTScore;
        space.vCandidates.push_back(c);
      }
    }
  }
  
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);  
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();
}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
ScaleSpace& ScaleSpace::operator=(const ScaleSpace &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.resize(rhs.im.size());
  copy(rhs.im, im);
  
  vFeatures = rhs.vFeatures;
  vFeaturesLUT = rhs.vFeaturesLUT;
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







