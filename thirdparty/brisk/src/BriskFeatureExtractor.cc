#include "BriskFeatureExtractor.h"
#include <cvd/integral_image.h>

using namespace std;
using namespace CVD;

const float BriskDescriptorExtractor::basicSize_    =12.0;
const unsigned int BriskDescriptorExtractor::scales_=64;
const float BriskDescriptorExtractor::scalerange_   =30;        // 40->4 Octaves - else, this needs to be adjusted...
const unsigned int BriskDescriptorExtractor::n_rot_ =1024;	 // discretization of the rotation look-up


// constructors
BriskDescriptorExtractor::BriskDescriptorExtractor(bool rotationInvariant,
		bool scaleInvariant, float patternScale){

	std::vector<float> rList;
	std::vector<int> nList;

	// this is the standard pattern found to be suitable also
	rList.resize(5);
	nList.resize(5);
	const double f=0.85*patternScale;

	rList[0]=f*0;
	rList[1]=f*2.9;
	rList[2]=f*4.9;
	rList[3]=f*7.4;
	rList[4]=f*10.8;

	nList[0]=1;
	nList[1]=10;
	nList[2]=14;
	nList[3]=15;
	nList[4]=20;

	rotationInvariance=rotationInvariant;
	scaleInvariance=scaleInvariant;
	generateKernel(rList,nList,5.85*patternScale,8.2*patternScale);

}
BriskDescriptorExtractor::BriskDescriptorExtractor(std::vector<float> &radiusList,
		std::vector<int> &numberList, bool rotationInvariant, bool scaleInvariant,
		float dMax, float dMin, std::vector<int> indexChange){
	rotationInvariance=rotationInvariant;
	scaleInvariance=scaleInvariant;
	generateKernel(radiusList,numberList,dMax,dMin,indexChange);
}

void BriskDescriptorExtractor::generateKernel(std::vector<float> &radiusList,
			std::vector<int> &numberList, float dMax, float dMin,
			std::vector<int> indexChange){

	dMax_=dMax;
	dMin_=dMin;

	// get the total number of points
	const int rings=radiusList.size();
	assert(radiusList.size()!=0&&radiusList.size()==numberList.size());
	points_=0; // remember the total number of points
	for(int ring = 0; ring<rings; ring++){
		points_+=numberList[ring];
	}
	// set up the patterns
	patternPoints_=new BriskPatternPoint[points_*scales_*n_rot_];
	BriskPatternPoint* patternIterator=patternPoints_;

	// define the scale discretization:
	static const float lb_scale=log(scalerange_)/log(2.0);
	static const float lb_scale_step = lb_scale/(scales_);

	scaleList_=new float[scales_];
	sizeList_=new unsigned int[scales_];

	const float sigma_scale=1.3;

	for(unsigned int scale = 0; scale <scales_; ++scale){
		scaleList_[scale]=pow((double)2.0,(double)(scale*lb_scale_step));
		sizeList_[scale]=0;

		// generate the pattern points look-up
		double alpha, theta;
		for(size_t rot=0; rot<n_rot_; ++rot){
			theta = double(rot)*2*M_PI/double(n_rot_); // this is the rotation of the feature
			for(int ring = 0; ring<rings; ++ring){
				for(int num=0; num<numberList[ring]; ++num){
					// the actual coordinates on the circle
					alpha = (double(num))*2*M_PI/double(numberList[ring]);
					patternIterator->x=scaleList_[scale]*radiusList[ring]*cos(alpha+theta); // feature rotation plus angle of the point
					patternIterator->y=scaleList_[scale]*radiusList[ring]*sin(alpha+theta);
					// and the gaussian kernel sigma
					if(ring==0){
						patternIterator->sigma = sigma_scale*scaleList_[scale]*0.5;
					}
					else{
						patternIterator->sigma = sigma_scale*scaleList_[scale]*(double(radiusList[ring]))*sin(M_PI/numberList[ring]);
					}
					// adapt the sizeList if necessary
					const unsigned int size=ceil(((scaleList_[scale]*radiusList[ring])+patternIterator->sigma))+1;
					if(sizeList_[scale]<size){
						sizeList_[scale]=size;
					}

					// increment the iterator
					++patternIterator;
				}
			}
		}
	}

	// now also generate pairings
	shortPairs_ = new BriskShortPair[points_*(points_-1)/2];
	longPairs_ = new BriskLongPair[points_*(points_-1)/2];
	noShortPairs_=0;
	noLongPairs_=0;

	// fill indexChange with 0..n if empty
	unsigned int indSize=indexChange.size();
	if(indSize==0) {
		indexChange.resize(points_*(points_-1)/2);
		indSize=indexChange.size();
	}
	for(unsigned int i=0; i<indSize; i++){
		indexChange[i]=i;
	}
	const float dMin_sq =dMin_*dMin_;
	const float dMax_sq =dMax_*dMax_;
	for(unsigned int i= 1; i<points_; i++){
		for(unsigned int j= 0; j<i; j++){ //(find all the pairs)
			// point pair distance:
			const float dx=patternPoints_[j].x-patternPoints_[i].x;
			const float dy=patternPoints_[j].y-patternPoints_[i].y;
			const float norm_sq=(dx*dx+dy*dy);
			if(norm_sq>dMin_sq){
				// save to long pairs
				BriskLongPair& longPair=longPairs_[noLongPairs_];
				longPair.weighted_dx=int((dx/(norm_sq))*2048.0+0.5);
				longPair.weighted_dy=int((dy/(norm_sq))*2048.0+0.5);
				longPair.i = i;
				longPair.j = j;
				++noLongPairs_;
			}
			else if (norm_sq<dMax_sq){
				// save to short pairs
				assert(noShortPairs_<indSize); // make sure the user passes something sensible
				BriskShortPair& shortPair = shortPairs_[indexChange[noShortPairs_]];
				shortPair.j = j;
				shortPair.i = i;
				++noShortPairs_;
			}
		}
	}

	// no bits:
	strings_=(int)ceil((float(noShortPairs_))/128.0)*4*4;
}


// simple alternative:
__inline__ int BriskDescriptorExtractor::smoothedIntensity(const CVD::Image<CVD::byte>& image,
		const CVD::Image<int>& integral,const float key_x,
			const float key_y, const unsigned int scale,
			const unsigned int rot, const unsigned int point) const{

	// get the float position
	const BriskPatternPoint& briskPoint = patternPoints_[scale*n_rot_*points_ + rot*points_ + point];
	const float xf=briskPoint.x+key_x;
	const float yf=briskPoint.y+key_y;
	const int x = int(xf);
	const int y = int(yf);
	const int& imagecols=image.size().y;

	// get the sigma:
	const float sigma_half=briskPoint.sigma;
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
		return (ret_val+512)/1024;
	}

	// this is the standard case (simple, not speed optimized yet):

	// scaling:
	const int scaling = 4194304.0/area;
	const int scaling2=float(scaling)*area/1024.0;

	// the integral image is larger:
	const int integralcols=imagecols+1;

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

	if(dx+dy>2){
		// now the calculation:
		unsigned char* ptr=(unsigned char*)image.data()+x_left+imagecols*y_top;
		// first the corners:
		ret_val=A*int(*ptr);
		ptr+=dx+1;
		ret_val+=B*int(*ptr);
		ptr+=dy*imagecols+1;
		ret_val+=C*int(*ptr);
		ptr-=dx+1;
		ret_val+=D*int(*ptr);

		// next the edges:
		int* ptr_integral=(int*)integral.data()+x_left+integralcols*y_top+1;
		// find a simple path through the different surface corners
		const int tmp1=(*ptr_integral);
		ptr_integral+=dx;
		const int tmp2=(*ptr_integral);
		ptr_integral+=integralcols;
		const int tmp3=(*ptr_integral);
		ptr_integral++;
		const int tmp4=(*ptr_integral);
		ptr_integral+=dy*integralcols;
		const int tmp5=(*ptr_integral);
		ptr_integral--;
		const int tmp6=(*ptr_integral);
		ptr_integral+=integralcols;
		const int tmp7=(*ptr_integral);
		ptr_integral-=dx;
		const int tmp8=(*ptr_integral);
		ptr_integral-=integralcols;
		const int tmp9=(*ptr_integral);
		ptr_integral--;
		const int tmp10=(*ptr_integral);
		ptr_integral-=dy*integralcols;
		const int tmp11=(*ptr_integral);
		ptr_integral++;
		const int tmp12=(*ptr_integral);

		// assign the weighted surface integrals:
		const int upper=(tmp3-tmp2+tmp1-tmp12)*r_y_1_i;
		const int middle=(tmp6-tmp3+tmp12-tmp9)*scaling;
		const int left=(tmp9-tmp12+tmp11-tmp10)*r_x_1_i;
		const int right=(tmp5-tmp4+tmp3-tmp6)*r_x1_i;
		const int bottom=(tmp7-tmp6+tmp9-tmp8)*r_y1_i;

		return (ret_val+upper+middle+left+right+bottom+scaling2/2)/scaling2;
	}

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

	return (ret_val+scaling2/2)/scaling2;
}

bool RoiPredicate(const float minX, const float minY,
		const float maxX, const float maxY, const KeyPoint& keyPt){
	const CVD::ImageRef& pt = keyPt.pt;
	return (pt.x < minX) || (pt.x >= maxX) || (pt.y < minY) || (pt.y >= maxY);
}

// computes the descriptor
void BriskDescriptorExtractor::compute(const CVD::Image<CVD::byte>& image,
		std::vector<KeyPoint>& keypoints, CVD::Image<CVD::byte>& descriptors) const{

	//Remove keypoints very close to the border
	unsigned int ksize=keypoints.size();
	std::vector<int> kscales; // remember the scale per keypoint
	kscales.resize(ksize);
	static const float log2 = 0.693147180559945;
	static const float lb_scalerange = log(scalerange_)/(log2);
	std::vector<KeyPoint>::iterator beginning = keypoints.begin();
	std::vector<int>::iterator beginningkscales = kscales.begin();
	static const float basicSize06=basicSize_*0.6;
	unsigned int basicscale=0;

	const CVD::ImageRef& imgsize = image.size();

	if(!scaleInvariance)
		basicscale=std::max((int)(scales_/lb_scalerange*(log(1.45*basicSize_/(basicSize06))/log2)+0.5),0);
	for(unsigned int k=0; k<ksize; k++){
		unsigned int scale;
		if(scaleInvariance){
			scale=std::max((int)(scales_/lb_scalerange*(log(keypoints[k].size/(basicSize06))/log2)+0.5),0);
			// saturate
			if(scale>=scales_) scale = scales_-1;
			kscales[k]=scale;
		}
		else{
			scale = basicscale;
			kscales[k]=scale;
		}
		const int border = sizeList_[scale];
		const int border_x=imgsize.y-border;
		const int border_y=imgsize.x-border;
		if(RoiPredicate(border, border,border_x,border_y,keypoints[k])){
			keypoints.erase(beginning+k);
			kscales.erase(beginningkscales+k);
			if(k==0){
				beginning=keypoints.begin();
				beginningkscales = kscales.begin();
			}
			ksize--;
			k--;
		}
	}

	// first, calculate the integral image over the whole image:
	// current integral image
	CVD::Image<int> _integral; // the integral image
	CVD::integral_image(image);

	int* _values=new int[points_]; // for temporary use

	// resize the descriptors:
	descriptors=CVD::Image<unsigned char>(image.size(), 0);

	// now do the extraction for all keypoints:

	// temporary variables containing gray values at sample points:
	int t1;
	int t2;

	// the feature orientation
	int direction0;
	int direction1;

	unsigned char* ptr = (unsigned char*)descriptors.data();
	for(unsigned int k=0; k<ksize; k++){
		int theta;
		KeyPoint& kp=keypoints[k];
		const int& scale=kscales[k];
		int shifter=0;
		int* pvalues =_values;
		const float& x=kp.pt.x;
		const float& y=kp.pt.y;
		if(true/*kp.angle==-1*/){
			if (!rotationInvariance){
				// don't compute the gradient direction, just assign a rotation of 0°
				theta=0;
			}
			else{
				// get the gray values in the unrotated pattern
				for(unsigned int i = 0; i<points_; i++){
					*(pvalues++)=smoothedIntensity(image, _integral, x,
							y, scale, 0, i);
				}

				direction0=0;
				direction1=0;
				// now iterate through the long pairings
				const BriskLongPair* max=longPairs_+noLongPairs_;
				for(BriskLongPair* iter=longPairs_; iter<max; ++iter){
					t1=*(_values+iter->i);
					t2=*(_values+iter->j);
					const int delta_t=(t1-t2);
					// update the direction:
					const int tmp0=delta_t*(iter->weighted_dx)/1024;
					const int tmp1=delta_t*(iter->weighted_dy)/1024;
					direction0+=tmp0;
					direction1+=tmp1;
				}
				kp.angle=atan2((float)direction1,(float)direction0)/M_PI*180.0;
				theta=int((n_rot_*kp.angle)/(360.0)+0.5);
				if(theta<0)
					theta+=n_rot_;
				if(theta>=int(n_rot_))
					theta-=n_rot_;
			}
		}
		else{
			// figure out the direction:
			//int theta=rotationInvariance*round((_n_rot*atan2(direction.at<int>(0,0),direction.at<int>(1,0)))/(2*M_PI));
			if(!rotationInvariance){
				theta=0;
			}
			else{
				theta=(int)(n_rot_*(kp.angle/(360.0))+0.5);
				if(theta<0)
					theta+=n_rot_;
				if(theta>=int(n_rot_))
					theta-=n_rot_;
			}
		}

		// now also extract the stuff for the actual direction:
		// let us compute the smoothed values
		shifter=0;

		//unsigned int mean=0;
		pvalues =_values;
		// get the gray values in the rotated pattern
		for(unsigned int i = 0; i<points_; i++){
			*(pvalues++)=smoothedIntensity(image, _integral, x,
					y, scale, theta, i);
		}

		// now iterate through all the pairings
		UINT32_ALIAS* ptr2=(UINT32_ALIAS*)ptr;
		const BriskShortPair* max=shortPairs_+noShortPairs_;
		for(BriskShortPair* iter=shortPairs_; iter<max;++iter){
			t1=*(_values+iter->i);
			t2=*(_values+iter->j);
			if(t1>t2){
				*ptr2|=((1)<<shifter);

			} // else already initialized with zero
			// take care of the iterators:
			++shifter;
			if(shifter==32){
				shifter=0;
				++ptr2;
			}
		}

		ptr+=strings_;
	}

	// clean-up
	//delete *_integral;
	delete [] _values;
}

BriskDescriptorExtractor::~BriskDescriptorExtractor(){
	delete [] patternPoints_;
	delete [] shortPairs_;
	delete [] longPairs_;
	delete [] scaleList_;
	delete [] sizeList_;
}