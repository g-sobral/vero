
#include "cansafe.h"

using namespace std;
using namespace verocarfreedomdefs;

namespace verocarfreedom
{

CANSafe::CANSafe() { init(1.0f); }

CANSafe::CANSafe(float period_s=1.0f){ init(period_s); }

/**
 * \brief object initialization
 * \li sets initial timestamp
 * \li empty id and answers stores.
 * \li initial state is RUN
 */
void CANSafe::init(float period) {
  monitor_period_s_=period;
  gettimeofday(&first_time_,NULL);
  last_time_ = getNow();
  state=RUN;
  answermap.clear();
  missingFrames.clear();
}//init

/**
 * \brief get a current timestamp, relative to the object creation
 */
double CANSafe::getNow() {
  struct timeval tp;
  gettimeofday(&tp,NULL);
  double now = (tp.tv_sec - first_time_.tv_sec) +  (tp.tv_usec / 1000000.0f); // it is not proper subtraction, it is just to zero the time in the beggining
  return now;
}//getNow

/**
 * \brief adds a new frame id to be monitored
 * @param id the id of the can_frame to be monitored
 * @param fr_answer a can_frame to be sent if the monitored can_frame is missing. Note, fr_answer tipically has a different id. All fields of fr_answer (id, size, content) are used.
 */
void CANSafe::addFrameToBeMonitored(int id, can_frame &fr_answer) {
  answermap[id]=fr_answer;
}//addFrameToBeMonitored

/**
 * \brief called when a can_frame is received. Erases its id from the list of pending frames.
 */
void CANSafe::gotFrame(int id) {
    missingFrames.erase(id);
} //gotFrame

/**
 * \brief checks for missing frames.
 * \par it checks the current timestap, therefore it may be called multiple times.
 * \par idea: call missingFrame, if it returns 0, it is ok, otherwise send fr_answer to the CAN bus. Call it again to check if there is another missing frame, until it retunrs 0.
 * @returns 0 if no frame is missing, or the id of a missing frame if there is a missing frame. If thereare more than one missing frame, only one id is returned. The called should call this method repeteadly until it returns 0.
 * @param fr_answer is only modified if there is a missing frame, when the funtion returns its id. Otherwise it is left undisturbed.
 */
int CANSafe::missingFrame(can_frame &fr_answer){
  int retOK=0;
  double curr_time = getNow();
  if ((curr_time - last_time_) > monitor_period_s_) { // time to check! last_time_ is too far ago.
    state=CHECK; // change state!
    last_time_=curr_time;
  }

  if (CHECK==state) {// check for missing frames
    if (missingFrames.empty()) { // everything OK, back to normal
      state=RUN; // normal state
      // initialize missingFrames with all monitored ids
      map<int, can_frame>::iterator it;
      for (it=answermap.begin(); it != answermap.end() ; it++) {
	missingFrames.insert((*it).first); // insert the id to be monitored
      }
      return retOK; // no problema
    } else { // problems! return the first problem
      set<int>::iterator it;
      it = missingFrames.begin();
      int id = *it;
      fr_answer = answermap[id]; // send back the answer
      missingFrames.erase(id);// assume the user will deal with the problem.
      return id; // returns the problem id.
    }
  }//if state
  return retOK;
}//missingFrame


} //namespace
