# Time when data was measured.
time timeStamp

# Reference value for the angle of the steered wheel [rad].
# This is the command currently executed by the vehicle.
float64 referenceSteerAngle

# Reference value for the forward speed of the steered wheel [m/s].
# The value is negative when the vehicle is to move backwards.
float64 referenceSpeed

float64 steerAngle

float64 x

float64 y

float64 yaw

float64 vlong

float64 dyaw

# differential whell speeds
float32 speedLeft
float32 speedRight
# differential whell speeds
float32 speedLeftFront
float32 speedRightFront
uint32[4] enc_missing # indicates if frames from the encoder are missing since the last reading.

# painel
int32 mode
bool CarOn
bool lights
bool buzz
bool strobo
bool brake

# bumpers
bool bumperFront
bool bumperRear

# if necesssary in the future we may also implement the power interface, but it looks too simple for that with just one batt.
float32 battVolt

# Potencia
int16 Potenciaesq
int16 Potenciadir
int8 ErroEsq
int8 ErroDir

# motor controllers
float32 ampereMotorLeft
float32 ampereMotorRight
int32 acelLeft
int32 acelRight
# steering control
float32 ampereSteering
bool endCourseSteering

# Is the hardware in a non-critical non-standard state?
bool hasWarnings
# Is the hardware in a critical non-standard state?
bool hasFaults

string warnFaultReason
