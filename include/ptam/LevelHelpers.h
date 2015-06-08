// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

// LevelHelpers.h - a few handy tools to ease using levels.
// The important thing is the XXXPos functions, which convert
// image positions from one level to another. Use these whenever
// transforming positions to ensure consistent operation!!

#ifndef __LEVEL_HELPERS_H
#define __LEVEL_HELPERS_H
#include <brisk/keyPoint.hpp>
#include <TooN/TooN.h>
using namespace TooN;
using namespace CVD;

// Set of global colours useful for drawing stuff:
// (These are filled in in KeyFrame.cc)
extern Vector<3> gavLevelColors[];

inline int LevelScale(int lvl){
	return 1 << lvl;
}

inline float LevelNPos(float pos, float scale, float offset) {
  return pos / scale - offset;
}

inline Point2f LevelNPos(Point2f point, float scale, float offset) {
  return Point2f(LevelNPos(point.x, scale, offset),
                 LevelNPos(point.y, scale, offset));
}

// 1-D transform from level zero to level N:
inline double LevelNPos(double dRootPos, int nLevel)
{
  return (dRootPos + 0.5) / LevelScale(nLevel) - 0.5;
}

// 2-D transform from level zero to level N:
inline Vector<2> LevelNPos(Vector<2> v2RootPos, int nLevel)
{
  Vector<2> v2Ans;
  v2Ans[0] = LevelNPos(v2RootPos[0], nLevel);
  v2Ans[1] = LevelNPos(v2RootPos[1], nLevel);
  return v2Ans;
}

#endif
