
#ifndef ESMOCV_MASK_H
#define ESMOCV_MASK_H

#include <opencv2/core/core.hpp>

using namespace cv;

namespace esmocv
{
  /**
   * \brief a simple mask creator with uses one every *step* pixels.
   * \par if step=1 it'll use the whole image (i.e., as if no mask was set)
   */
  class StepMaskCreator {
  public:
    StepMaskCreator(int step) : step_(step) {}; 
    StepMaskCreator() : step_(3) {}; /// default constructor with step 3 
    cv::Mat createMask(const cv::Rect &ROI);
    
    
  protected:
    int step_;
    
  };//class mask
  
  
}// namespace


#endif 