#include "BriskLayer.h"

#include <cvd/vision.h>
#include <agast/agast5_8.h>
#include <agast/agast_corner_detect.h>

using namespace CVD;
using namespace std;

// construct a layer
BriskLayer::BriskLayer(const CVD::Image<CVD::byte>& img, float scale, float offset) {
	img_=img;
	scores_= CVD::Image<unsigned char>(img_.size(), 0);
	// attention: this means that the passed image reference must point to persistent memory
	scale_=scale;
	offset_=offset;

	CVD::ImageRef imsize= img_.size();
}
// derive a layer
BriskLayer::BriskLayer(const BriskLayer& layer, int mode){
	if(mode==LayerTypes::HALFSAMPLE){
		img_.resize(layer.img().size()/2);
		halfSample(layer.img(), img_);
		scale_= layer.scale()*2;
		offset_=0.5*scale_-0.5;
	}
	else {
		img_.resize(2*layer.img().size()/3);
		twoThirdsSample(layer.img(), img_);
		scale_= layer.scale()*1.5;
		offset_=0.5*scale_-0.5;
	}
	scores_= CVD::Image<unsigned char>(img_.size(), 0);

	CVD::ImageRef imsize= img_.size();
	// create an agast detector
	oastDetector_ = new agast::OastDetector9_16(imsize.x, imsize.y, 0);
	agastDetector_5_8_ = new agast::AgastDetector5_8(imsize.x, imsize.y, 0);
}

// Fast/Agast
// wraps the agast class
void BriskLayer::getAgastPoints(uint8_t threshold, std::vector<CVD::ImageRef>& keypoints){
	oastDetector_->set_threshold(threshold);
	std::vector<CvPoint> keypointstmp;
	oastDetector_->detect((unsigned char*)img_.data(),keypointstmp);
	keypoints.reserve(keypointstmp.size());
	for(size_t i=0;i<keypoints.size();++i) //convert to imageRefs
	{
		keypoints.push_back(CVD::ImageRef(keypointstmp.at(i).x, keypointstmp.at(i).y));
	}
	keypointstmp.clear();

	// also write scores
	const int num=keypoints.size();
	const int imcols=img_.size().y;

	for(int i=0; i<num; i++){
		const int offs=keypoints[i].x+keypoints[i].y*imcols;
		*((unsigned char*)scores_.data()+offs)=oastDetector_->cornerScore((unsigned char*)img_.data()+offs);
	}
}

uint8_t BriskLayer::getAgastScore(int x, int y, uint8_t threshold){
	const int& imagecols=img_.size().y;
	const int& imagerows=img_.size().x;
	if(x<3||y<3) return 0;
	if(x>=imagecols-3||y>=imagerows-3) return 0;
	uint8_t& score=*((unsigned char*)scores_.data()+x+y*imagecols);
	if(score>2) { return score; }
	oastDetector_->set_threshold(threshold-1);
	score = oastDetector_->cornerScore((unsigned char*)img_.data()+x+y*imagecols);
	if (score<threshold) score = 0;
	return score;
}

uint8_t BriskLayer::getAgastScore_5_8(int x, int y, uint8_t threshold){
	const int& imagecols=img_.size().y;
	const int& imagerows=img_.size().x;
	if(x<2||y<2) return 0;
	if(x>=imagecols-2||y>=imagerows-2) return 0;
	agastDetector_5_8_->set_threshold(threshold-1);
	uint8_t score = agastDetector_5_8_->cornerScore((unsigned char*)img_.data()+x+y*imagecols);
	if (score<threshold) score = 0;
	return score;
}

uint8_t BriskLayer::getAgastScore(float xf, float yf, uint8_t threshold, float scale){
	if(scale<=1.0f){
		// just do an interpolation inside the layer
		const int x=int(xf);
		const float rx1=xf-float(x);
		const float rx=1.0f-rx1;
		const int y=int(yf);
		const float ry1=yf-float(y);
		const float ry=1.0f-ry1;

		return rx*ry*getAgastScore(x, y, threshold)+
				rx1*ry*getAgastScore(x+1, y, threshold)+
				rx*ry1*getAgastScore(x, y+1, threshold)+
				rx1*ry1*getAgastScore(x+1, y+1, threshold);
	}
	else{
		// this means we overlap area smoothing
		const float halfscale = scale/2.0f;
		// get the scores first:
		for(int x=int(xf-halfscale); x<=int(xf+halfscale+1.0f); x++){
			for(int y=int(yf-halfscale); y<=int(yf+halfscale+1.0f); y++){
				getAgastScore(x, y, threshold);
			}
		}
		// get the smoothed value
		return value(scores_,xf,yf,scale);
	}
}

// access gray values (smoothed/interpolated)
__inline__ uint8_t BriskLayer::value(const CVD::Image<unsigned char>& mat, float xf, float yf, float scale){
	// get the position
	const int x = floor(xf);
	const int y = floor(yf);
	const CVD::Image<unsigned char>& image=mat;
	const int& imagecols=image.size().y;
	// get the sigma_half:
	const float sigma_half=scale/2;
	const float area=4.0*sigma_half*sigma_half;
	// calculate output:
	int ret_val;
	if(sigma_half<0.5){
		//interpolation multipliers:
		const int r_x=(xf-x)*1024;
		const int r_y=(yf-y)*1024;
		const int r_x_1=(1024-r_x);
		const int r_y_1=(1024-r_y);
		unsigned char* ptr=(unsigned char*)image.data()+x+y*imagecols;
		// just interpolate:
		ret_val=(r_x_1*r_y_1*int(*ptr));
		ptr++;
		ret_val+=(r_x*r_y_1*int(*ptr));
		ptr+=imagecols;
		ret_val+=(r_x*r_y*int(*ptr));
		ptr--;
		ret_val+=(r_x_1*r_y*int(*ptr));
		return 0xFF&((ret_val+512)/1024/1024);
	}

	// this is the standard case (simple, not speed optimized yet):

	// scaling:
	const int scaling = 4194304.0/area;
	const int scaling2=float(scaling)*area/1024.0;

	// calculate borders
	const float x_1=xf-sigma_half;
	const float x1=xf+sigma_half;
	const float y_1=yf-sigma_half;
	const float y1=yf+sigma_half;

	const int x_left=int(x_1+0.5);
	const int y_top=int(y_1+0.5);
	const int x_right=int(x1+0.5);
	const int y_bottom=int(y1+0.5);

	// overlap area - multiplication factors:
	const float r_x_1=float(x_left)-x_1+0.5;
	const float r_y_1=float(y_top)-y_1+0.5;
	const float r_x1=x1-float(x_right)+0.5;
	const float r_y1=y1-float(y_bottom)+0.5;
	const int dx=x_right-x_left-1;
	const int dy=y_bottom-y_top-1;
	const int A=(r_x_1*r_y_1)*scaling;
	const int B=(r_x1*r_y_1)*scaling;
	const int C=(r_x1*r_y1)*scaling;
	const int D=(r_x_1*r_y1)*scaling;
	const int r_x_1_i=r_x_1*scaling;
	const int r_y_1_i=r_y_1*scaling;
	const int r_x1_i=r_x1*scaling;
	const int r_y1_i=r_y1*scaling;

	// now the calculation:
	unsigned char* ptr=(unsigned char*)image.data()+x_left+imagecols*y_top;
	// first row:
	ret_val=A*int(*ptr);
	ptr++;
	const unsigned char* end1 = ptr+dx;
	for(; ptr<end1; ptr++){
		ret_val+=r_y_1_i*int(*ptr);
	}
	ret_val+=B*int(*ptr);
	// middle ones:
	ptr+=imagecols-dx-1;
	unsigned char* end_j=ptr+dy*imagecols;
	for(; ptr<end_j; ptr+=imagecols-dx-1){
		ret_val+=r_x_1_i*int(*ptr);
		ptr++;
		const unsigned char* end2 = ptr+dx;
		for(; ptr<end2; ptr++){
			ret_val+=int(*ptr)*scaling;
		}
		ret_val+=r_x1_i*int(*ptr);
	}
	// last row:
	ret_val+=D*int(*ptr);
	ptr++;
	const unsigned char* end3 = ptr+dx;
	for(; ptr<end3; ptr++){
		ret_val+=r_y1_i*int(*ptr);
	}
	ret_val+=C*int(*ptr);

	return 0xFF&((ret_val+scaling2/2)/scaling2/1024);
}