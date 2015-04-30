
#ifndef CANSAFE_H
#define CANSAFE_H

#include <map>
#include <set>
#include <verocarfreedomdefs/carfreedom_defs.h>
//#include <hydrocarfreedomdefs/carfreedom_defs.h>
#include <sys/time.h> // gettimeofday, timeval

using namespace std;

namespace verocarfreedom
{
  enum CANSafestate {
    RUN,
    CHECK
  };

/**
 * \brief monitors a given set of can frames timestamps. If some monitored can_frame is missing, an appropriate answer frame is returned.
 */
class CANSafe
{

  public:
    CANSafe(); /// uses the default period of 1.0s
    CANSafe(float period_s); /// alternative constructor, to choose a different period
    ~CANSafe() { };
    void addFrameToBeMonitored(int id, can_frame &fr_answer);
    void gotFrame(int id);
    int missingFrame(can_frame &fr_answer);


  private:
    void init(float period);/// initialization fc, called by constructor
    double getNow();
    CANSafestate state; /// two states: RUN - normal operation ; CHECK:
    double last_time_; /// timestamps of last check
    timeval first_time_; /// timestamp of object creation
    double monitor_period_s_; /// the period between frames allowed. monitored frames must have larger frequencies than this
    map<int, can_frame> answermap; /// stores a can_frame to be sent for each missing frame
    set<int> missingFrames; ///the ids of all monitored can_frames

}; //class

} // namespace

#endif

