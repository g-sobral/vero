#include "CtrlFactory.h"

 
namespace dvs
{

  CtrlFactory* CtrlFactory::instance=NULL;/// just initialization 
  
  /// if you have a new subclass, define here when it is created.
  Ctrl* CtrlFactory::newCtrl(std::string &algorithm, ros::NodeHandle &nh) {
    if ((algorithm == "dvs") ) {
        return new DvsCtrl(nh);
    }
    if ((algorithm == "ddvs") ) {
        return new DDvsCtrl(nh);
    }
    ROS_ERROR("CtrlFactory:: unknown algorithm name: %s", algorithm.c_str());
    return NULL;
  } 
  
}//namespace  