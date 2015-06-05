#include "ptam/MatchFinder.h"
#include "brisk/brisk.h"
#include <vector>
using namespace std;
using namespace CVD;

MatchFinder::MatchFinder(int maxDistance) {
  mMaxDistance = maxDistance;
}

bool MatchFinder::FindMatchCoarse(KeyFrame& kf, const Feature& ft, Feature*& ft2, unsigned int range) {
  bool mbFound = false;

  // cout << "Position of feature to track ";
  // cout << " x: " << ft.ptRootPos.x();
  // cout << " y: " << ft.ptRootPos.y();
  // cout << endl;
  //check indexer LUT
  Image<CVD::byte> im = kf.pyramid[0].im;
  Point2f pt = ft.ptRootPos;
  // Bounding box of search circle
  int nTop = (int)pt.y() - range;
  int nBottomPlusOne = (int)pt.y() + range + 1;
  int nLeft = (int)pt.x() - range;
  int nRight = (int)pt.x() + range;

  // Some bounds checks on the bounding box..
  if (nTop < 0)
    nTop = 0;
  if (nTop >= im.size().y)
    return false;
  if (nBottomPlusOne <= 0)
    return false;

  // The next section finds all the brisk features in all target level which
  // are near enough the search center. It's a bit optimised to use
  // a corner row look-up-table, since otherwise the routine
  // would spend a long time trawling throught the whole list of features!
  std::vector<Feature>::iterator i;
  std::vector<Feature>::iterator i_end;

  i = kf.vFeatures.begin() + kf.vFeaturesLUT[nTop];

  if (nBottomPlusOne >= im.size().y)
    i_end = kf.vFeatures.end();
  else
    i_end = kf.vFeatures.begin() + kf.vFeaturesLUT[nBottomPlusOne];

  Feature* ftBest;             // Best match so far
  int nBestDist = 10*mMaxDistance + 1; // Best score so far is beyond the max allowed
  float maxRange = range * range;
  float nBestMag = maxRange;

  for (; i < i_end; i++)       // For each corner ...
  {  
    // cout << "Compare with: ";
    // cout << " x: " << (*i).ptRootPos.x();
    // cout << " y: " << (*i).ptRootPos.y();
    // cout << endl;

    if ( (*i).ptRootPos.x() < nLeft || (*i).ptRootPos.x() > nRight)
      continue;

    float cMag=(ft.ptRootPos - (*i).ptRootPos).mag_squared();
    if ( cMag > maxRange)
      continue;              // ... reject all those not close enough..

    static CVD::HammingSse hamming;
    int dist = hamming.getScore((*i).descriptor, ft.descriptor);

    if (dist < nBestDist)     // Best yet?
    {
      ftBest = &(*i);
      nBestDist = dist;
      nBestMag=cMag;
    }else if(dist==nBestDist && cMag<nBestMag){
      ftBest = &(*i);
      nBestDist = dist;
      nBestMag=cMag; 
    }
  } // done looping over corners

  if (nBestDist < mMaxDistance)     // Found a valid match?
  {
    //hack to remove const-ness of type
    ft2=ftBest;
    // ft2 = const_cast<Feature *>(ftBest);    
    mbFound = true;
  }
  else{
    mbFound = false;
  }
  return mbFound;
}