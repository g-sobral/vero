
#include "mask.h"


namespace esmocv
{
  /// @returns the created mask with ROI size and data type FLOAT.
  cv::Mat StepMaskCreator::createMask(const cv::Rect &ROI) {
    cv::Mat mask = Mat::zeros(ROI.width,ROI.height,CV_32F);
  
    long count=0;
    MatIterator_<float> it = mask.begin<float>(), it_end = mask.end<float>();
    // iterates from the begining to the end of the matrix
    for(; it != it_end; ++it) {
      if (0==(count % step_))
        *it = 1.0f;
      count ++;
    }//for
    return mask;
  }//createMask


}//namespace
  