
#ifndef HYDRO_CARFREEDOM_H
#define HYDRO_CARFREEDOM_H

#include "verocarfreedomdefs/carfreedom.h"
#include <candriver.h>
#include "verocarfreedomdefs/carfreedom_defs.h"
#include <vector>
//#include <gbxutilacfr/mathdefs.h> // CLIP_TO_LIMITS, DEG2RAD
//#include <hydrobros1/types.h> // Frame2d
#include <sockutil/dvsutil.h>

using namespace std;

namespace verocanprint
{
/**
 * \brief inherits from hydrosocketcan::CANDriver, with the additional functionality of pretty-printing the can_frames from/to the CarFreedom
 * \par This class is specific for the carfreedom. It can be used as a hydro driver, although there is not a corresponding hydrointerface
 * \par For each can_frame, prints a abbreviated text with its type, and the most important data fields, besides the timestamp, id, and data bytes
 */
class Driver : public socketcan_lib::CANDriver
{
public:

    Driver( const socketcan_lib::Context& context);

    virtual ~Driver();
    virtual int decode(struct can_frame frame);
  protected:
    verocarfreedomdefs_msgs::CarData data_;/// used to store the decoded data from each received can_frame
}; // class

/*
// Used for dynamically loading driver. CANDriver is the generic type.
class Factory : public hydrosocketcan::CanFactory
{
public:
    hydrosocketcan::CANDriver*
    createDriver( const hydroutil::Context &context ) const
    {
        return new hydrocanprint::Driver( context );
    }
};
*/
} // namespace
/*
// Used for dynamically loading driver
extern "C" {
    //hydrocanprint::Factory *createDriverFactory();
    verohydrointerfaces::CANInterfaceFactory *createDriverFactory();
}
*/
#endif



