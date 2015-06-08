#include "ptam/VideoSource.h"
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>

using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace cv;

static int OPENCV_VIDEO_W, OPENCV_VIDEO_H;
static const char DEFAULT_VIDEO_SOURCE[] = "/dev/video0";

VideoSource::VideoSource()
{
  /*!
     * Retrieve the name of the video source file from settings.cfg.
     */
  string videoSourceFile = GVars3::GV2.GetString("VideoFilename",
                           DEFAULT_VIDEO_SOURCE);

  cout << "  VideoSource_Linux: Opening video source..." << endl;

  string cmp(DEFAULT_VIDEO_SOURCE);
  if (videoSourceFile.compare(cmp) == 0) {
    OPENCV_VIDEO_W = 640;
    OPENCV_VIDEO_H = 480;
    cout << "Streaming from device..." << endl;
    mptr = new VideoCapture(0);
  }
  else {
    //Can change here the resize of the frame if need be
    OPENCV_VIDEO_W = 1920 / 2;
    OPENCV_VIDEO_H = 1080 / 2;
    mptr = new VideoCapture(videoSourceFile);
    cout << " Streaming from file..." << endl;
  }

  VideoCapture* cap = (VideoCapture*)mptr;
  if (!cap->isOpened()) {
    cerr << "Unable to get the camera" << endl;
    exit(-1);
  }
  cout << "  ... got video source." << endl;
  mirSize = ImageRef(OPENCV_VIDEO_W, OPENCV_VIDEO_H);
};

ImageRef VideoSource::Size()
{
  return mirSize;
};

void conversionNB(Mat frame, Image<byte> &imBW) {
  Mat clone = frame.clone();
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++) {
    for (int j = 0; j < OPENCV_VIDEO_W; j++) {
      imBW[i][j] = (frame_p(i, j)[0] + frame_p(i, j)[1] + frame_p(i, j)[2]) / 3;
    }
  }

}

void conversionRGB(Mat frame, Image<Rgb<byte> > &imRGB) {
  Mat clone = frame.clone();
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++) {
    for (int j = 0; j < OPENCV_VIDEO_W; j++) {
      imRGB[i][j].red = frame_p(i, j)[2];
      imRGB[i][j].green = frame_p(i, j)[1];
      imRGB[i][j].blue = frame_p(i, j)[0];
    }
  }
}

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  Mat frame;
  VideoCapture* cap = (VideoCapture*)mptr;
  *cap >> frame;
  // conversionNB(frame, imBW);
  // conversionRGB(frame, imRGB);
  Mat ftmp;//dst image
  cv::resize(frame, ftmp, cv::Size(OPENCV_VIDEO_W, OPENCV_VIDEO_H)); //resize image
  conversionNB(ftmp, imBW);
  conversionRGB(ftmp, imRGB);
}

