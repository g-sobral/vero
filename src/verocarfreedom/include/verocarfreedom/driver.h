
#ifndef HYDRO_CARFREEDOM_H
#define HYDRO_CARFREEDOM_H

#include "ros/ros.h"
#include <verocarfreedomdefs_msgs/CarData.h>
#include <verocarfreedomdefs_msgs/CarCommand.h>
#include <verocarfreedomdefs/carfreedom.h>
#include <candriver.h>
#include <verocarfreedomdefs/carfreedom_defs.h>
#include <vector>
#include <set>
//#include <gbxutilacfr/mathdefs.h> // CLIP_TO_LIMITS, DEG2RAD, MIN, MAX
#include <sockutil/dvsutil.h>
//#include <hydrobros1/types.h> // Frame2d
#include "encodermodel.h"
#include "cansafe.h"
#include <sys/time.h> // gettimeofday

using namespace std;

namespace verocarfreedom
{

typedef struct Context_ {
	int CAN_Delay_Between_Queries_us;
	int Encoder_Pulses_Period_ms;
	double MaxSteerAngleLeft;
	double MaxSteerAngleRight;
	vector<double>SteeringAng2CmdEnd1stLine_Adeg_Cmd;
	vector<double>SteeringAng2CmdStart2ndLine_Adeg_Cmd;
	double SteeringAng2Cmd_ZeroCmd;
	double MaxForwardSpeed;
	double MaxReverseSpeed;
	double JoystickMaxForward;
	double JoystickMaxReverse;
	double JoystickMaxLeft;
	double JoystickMaxRight;
	double WheelRadius_m;
	double InterAxesDistance_m;
	double AxisLength_m;
	double Wheels_V_Uncertainty;
	double PulsesPer360;
	double SteeringEncoderMaxLeft;
	double SteeringEncoderMaxRight;
	int Encoder_FrontLeft_Invert;
	int Encoder_FrontRight_Invert;
	int Encoder_RearLeft_Invert;
	int Encoder_RearRight_Invert;
	int PrintCanFrames;
	int WriteCommandsRPH;
	int EnableMotion;
} Context;

void IniciaContext (Context &context);

class EncodersModel;

typedef set<unsigned int, less<unsigned int> > setuint;

/**
 * \li Test encoder and command values with new conversions
 * \li Test if encoder programming messages are resent when ecoder fails
 * \li Test if odometry outputs correctly linear and rotational speeds
 */
class Driver : public /*verocarfreedomdefs::CarFreedom,*/ socketcan_lib::CANDriver
{
public:

    Driver(Context context, socketcan_lib::Context socket_context);

    virtual ~Driver();

    virtual void enable();

    virtual void read( verocarfreedomdefs_msgs::CarData &data );

    virtual void write( verocarfreedomdefs_msgs::CarCommand& command );

    virtual void getStatus( std::string &status, bool &isWarn, bool &isFault );

    virtual verocarfreedom::Capabilities capabilities() const;

    virtual int decode(struct can_frame frame);

	 void requery_now() {requery_now_ = true;} /// resend the queries.
  protected:
    void check_encoder_counters(int i,unsigned char encoder_counter);
    void updateOdometry();
    std::vector<unsigned int> queries2send; /// stores the ids of all queries to be send at each period
    setuint pendingAnswers; /// stores the ids of the queries sent and not answered. use this to implement error-checking, or to re-send some can frames.
    unsigned char encoder_pulses_period; /// period, in ms, to configrue the encoders to send pulse counts.
    verocarfreedom::Capabilities capabilities_;
    verocarfreedomdefs_msgs::CarData *data_;
    verocarfreedom::Config cfg_;
    unsigned int querydelay_;/// if there are timing problems in the can bus, set a delay between each query.
	 bool requery_now_; /// when set to true, the driver will send the queries.
    int enableMotion_; /// if zero, everything works except that motor and steering commands are not sent.
    EncodersModel *encoders_;
    unsigned int lastcmd_steeringAngle, lastcmd_motorspd;

    unsigned int mps2rph(float speed_mps);
    float steeringCmd2Angle(float stcmd) ;
    void differentialDrive(float stangle, float &cmdr, float &cmdl) ;

    double last_tstamp_; /// to calculate instantaneous speed, it needs the dt, so it stores the last timestamp to compare with the current one.
    CANSafe cansafe;
    std::vector<double> Ang2Cmd_End1stLine_;/// a point, [0] is angle deg and [1] is command
    std::vector<double> Ang2Cmd_Start2ndLine_;/// a point, [0] is angle deg and [1] is command
    double ZeroSteerCmd_; /// a failsafe zero command, if the conversion lines are not weel defined
}; // class

/*
// Used for dynamically loading driver
class Factory : public hydrointerfaces::CarFreedomFactory
{
public:
    hydrointerfaces::CarFreedom*
    createDriver( const hydroutil::Context &context ) const
    {
        return new Driver( context );
    }
};
*/
} // namespace
/*
// Used for dynamically loading driver
extern "C" {
    hydrointerfaces::CarFreedomFactory *createCarDriverFactory();
}
*/
#endif



