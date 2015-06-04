#include "ptam/MatchFinder.h"
#include "brisk/brisk.h"
#include <vector>
using namespace std;
using namespace CVD;

MatchFinder::MatchFinder(int maxDistance) {
  mMaxDistance = maxDistance;
}

bool MatchFinder::FindMatchCoarse(const KeyFrame& kf, const Feature& ft, Feature& ft2, unsigned int range) {
  bool mbFound = false;

  Image<CVD::byte> im = kf.pyramid[0].im;
  Point2f pt = ft.ptRootPos;
  // Bounding box of search circle
  float nTop = pt.y() - range;
  float nBottomPlusOne = pt.y() + range + 1;
  float nLeft = pt.x() - range;
  float nRight = pt.x() + range;

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
  std::vector<Feature>::const_iterator i;
  std::vector<Feature>::const_iterator i_end;

  i = kf.vFeatures.begin() + kf.vFeaturesLUT[nTop];

  if (nBottomPlusOne >= im.size().y)
    i_end = kf.vFeatures.end();
  else
    i_end = kf.vFeatures.begin() + kf.vFeaturesLUT[nBottomPlusOne];

  Feature ftBest;             // Best match so far
  int nBestDist = mMaxDistance + 1; // Best score so far is beyond the max allowed

  float maxRange = range * range;

  for (; i < i_end; i++)       // For each corner ...
  {
    if ( (*i).ptRootPos.x() < nLeft || (*i).ptRootPos.x() > nRight)
      continue;

    if ( (ft.ptRootPos - (*i).ptRootPos).mag_squared() > maxRange)
      continue;              // ... reject all those not close enough..

    static CVD::HammingSse hamming;
    int dist = hamming.getScore((*i).descriptor, ft.descriptor);

    if (dist < nBestDist)     // Best yet?
    {
      ftBest = *i;
      nBestDist = dist;
    }
  } // done looping over corners

  if (nBestDist < mMaxDistance)     // Found a valid match?
  {
    ft2 = ftBest;
    mbFound = true;
  }
  else
    mbFound = false;
  return mbFound;
}