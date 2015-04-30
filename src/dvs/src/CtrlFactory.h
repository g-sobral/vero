#ifndef DVS_CTRL_FACTORY_H
#define DVS_CTRL_FACTORY_H
#include "Ctrl.h"
#include "DvsCtrl.h"
#include "DDvsCtrl.h"

namespace dvs
{
/**
 * \brief factory method implemented as a singleton. Ask the singleton to create a socket client for you. 
 */
class CtrlFactory {
private:
  static CtrlFactory* instance;
protected:
  CtrlFactory(){}
public:
  
  static CtrlFactory* uniqueInstance() {
    if (NULL==instance) instance = new CtrlFactory();
    return instance;
  }//uniqueInstance
  
  Ctrl* newCtrl(std::string &algorithm, ros::NodeHandle &nh) ;
  
}; //class

}//namespace
#endif