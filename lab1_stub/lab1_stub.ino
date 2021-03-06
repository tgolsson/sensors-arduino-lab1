//TODO: change this varriable to 1 once you are ready to compile with rosserial_arduino support
#define ROS_SUPPORT 1

#if ROS_SUPPORT
#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <arduino_pkg/MotorState.h>
#include <arduino_pkg/SetPosition.h>
#include <arduino_pkg/SetPID.h>
#include <arduino_pkg/SetVelocity.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#endif

const int
ENCODER_PIN = 7,
    TOLERANCE = 3;
const double
    KP = 50.0,
    KI = 0,
    KD = 0.0,
    V_MAX = 280.0,
    TICKS_ROTATION = 235.0,
        TIME_SCALE = 1e6; // 1-over-micro; as time is counted in microseconds
   
//======================= Struct Definitions ======================
//this struct holds the state values of a desired control point
struct ControlStates
{
    float r_; //setpoint value at current iteration
    float rf_; //final setpoint value
    float ri_; //initial setpoint value
    float e_; //error
    float de_; //error derivative

    float ti_; //time when we initialized motion (seconds)
    float T_; //time for executing the loop

    float u_; //computed control

    bool active_; //flag indicating whether the corresponding controller is active or not

    bool on_; 
ControlStates(float r, float rf, float ri, float e, float de, float ti, float T, int u, bool active) : r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), active_(active) {};
};

//this struct holds the variables for a PID controller
struct PIDParameters {
    float Kp_;
    float Kd_;
    float Ki_;
    float u_max_; //Maximum controller output (<= max PWM)
    float u_min_; //Minimum controller output [>= -(max PWM)]
    float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

//this struct holds the pin numbers for the motor shield
struct MotorShieldPins {
    int DIR_; //direction pin
    int PWM_; //pwm pin
    int BRK_; //brake pin
    int CUR_; //current sensor

MotorShieldPins(int DIR, int BRK, int PWM_pin, int CUR) : DIR_(DIR), BRK_(BRK), PWM_(PWM_pin), CUR_(CUR) {};
};

//this struct holds the pin for the encoder and the current number of ticks
struct EncoderStates
{
    int ENC_; //pin for the encoder
    int p_;   //current encoder pulses
    int pp_; //previous p
    float dp_;//time-derivative of the encoder pulses
    
EncoderStates(int PIN, int pos) : ENC_(PIN), p_(pos) {
    dp_ = 0;
};
};

/////////////////Function Declarations//////////////////
//function to execute upon change of the encoder pin
void readEncoder();
//this function writes a desired control value to the motor
void actuate(float control, MotorShieldPins *mps);
//this function performs position control using the PID and minimumJerk
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p);
//this function performs velocity control using the PID and minimumJerk
void velocityControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p);
//this function computes the next setpoint to obtain a mnimum jerk trajectory
float minimumJerk(float t0, float t, float T, float q0, float qf);
//this function computes controls with a PID
float pid(float e, float de, PIDParameters* p);
//this function should update the motor state variable from the current measured values
void updateState();

// initializes the motor pins
void initMotor(MotorShieldPins* pins);

#if ROS_SUPPORT
/////////////////ROS Callbacks//////////////////////////
//callback function for setting a new position
void setPosCallback(const arduino_pkg::SetPosition::Request &req, arduino_pkg::SetPosition::Response &res);
//callback function for setting new PID parameters
void setPIDCallback(const arduino_pkg::SetPID::Request &req, arduino_pkg::SetPID::Response &res);
//callback function for setting a new velocities
void setVelCallback(const arduino_pkg::SetVelocity::Request &req, arduino_pkg::SetVelocity::Response &res);
//callback function for switching off the motor brake
void setOn(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
//callback function for switching on the motor brake
void setOff(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
///////////////////////////////////////////////////////////////////////////////////

///////////////////ROS global variables/////////////////////////
ros::NodeHandle nh;
arduino_pkg::MotorState state;
std_msgs::Float32 r;
std_msgs::Float32 control;
ros::Publisher state_publisher("/motor_state", &state);
ros::Publisher r_publisher("/r", &r);
ros::Publisher control_publisher("/c", &control);
ros::ServiceServer<arduino_pkg::SetPosition::Request, arduino_pkg::SetPosition::Response> pos_server("set_pos", &setPosCallback);
ros::ServiceServer<arduino_pkg::SetVelocity::Request, arduino_pkg::SetVelocity::Response> vel_server("set_vel", &setVelCallback);
ros::ServiceServer<arduino_pkg::SetPID::Request, arduino_pkg::SetPID::Response> pid_server("set_pid", &setPIDCallback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> on_server("set_on", &setOn);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> off_server("set_off", &setOff);
///////////////////////////////////////////////////////////////////////////////////////////////
#endif

//////////////Global Vars////////////
//PWM resoluion: 12 bit, i.e., 0 - 4095
const float pwm_resolution = 4095;
//storage for timers
int t_new, t_old, t_old_serial;
//sampling time in microseconds
int dT = 1000;
int dT_serial = 50000;
//storage of the electric current through the motors
float current;

//TODO: instantiate these structs to the correct PIN values
MotorShieldPins *motor1 = new MotorShieldPins(12, 9, 3, A0);
ControlStates *mc1 = new ControlStates(0, 0, 0, 0, 0, 0, 0, 0, false);
PIDParameters *pid_mc1 = new PIDParameters(KP,KI,KD, pwm_resolution, -pwm_resolution, 0);
EncoderStates *enc1 = new EncoderStates(ENCODER_PIN, 0);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

    t_old = micros();
    t_old_serial = micros();

#if ROS_SUPPORT == 0
    Serial.begin(9600);
#endif
    analogWriteResolution(12);
    analogReadResolution(12);

    initMotor(motor1);

    pinMode(ENCODER_PIN,INPUT);
    digitalWrite(ENCODER_PIN,HIGH); // pull-up
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), readEncoder, RISING);

    analogWrite(motor1->PWM_,4095);

#if ROS_SUPPORT == 0
    digitalWrite(motor1->PWM_,HIGH);
    for (int i = 0; i < 10; i++){
        int startP = enc1->p_;
        long start = micros();
        double    seconds = 0;
        while( seconds < 10)
        {
            seconds = (micros()-start)/(1e6);
            Sleep(0);
            
        }
        Serial.println(enc1->p_ - startP);
    }
#endif
    //Setup ROS related variables

#if ROS_SUPPORT
    ////// ROS initializations////
    nh.initNode();
    nh.advertise(state_publisher);
    nh.advertise(r_publisher); 
    nh.advertise(control_publisher); 
    nh.advertiseService(pos_server);
    nh.advertiseService(vel_server);
    nh.advertiseService(pid_server);
    nh.advertiseService(on_server);
    nh.advertiseService(off_server);
    ///////////////////////////////
#endif
}

void readEncoder() {
    if (digitalRead(motor1->DIR_))
        enc1->p_++;
    else
        enc1->p_--;
}

void actuate(float control, MotorShieldPins *mps)
{
    // Set motor direction based on sign of control value
    digitalWrite(mps->DIR_, control < 0 ? LOW : HIGH);

    // Force to positive and make sure it is in allowed control range
    double controlNew = abs(control);
    controlNew = max(min(pwm_resolution, controlNew), 0);

    // Write to pin
    analogWrite(mps->PWM_, controlNew);
}
//--------------------------------------------------------------------------
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p)
{
    // Sliding sum for encoder speed
    enc1->dp_ = enc1->dp_ * 0.99 + 0.01*(enc1->p_ - enc1->pp_)/(dT/TIME_SCALE);  // dT = 1000 us = 0.001 s
    enc1->pp_ = enc1->p_;

    // Early escape to prevent "almost-there" PWM-hum
    if (abs(enc1->p_ - c_s->rf_) < TOLERANCE)
    {
        c_s->u_ = 0;
        return;
    }

    // Set-point calculation
    double setPoint = minimumJerk(c_s->ti_, (double)t_new, c_s->T_, c_s->ri_, c_s->rf_);
    c_s->r_ = setPoint;

    // Error and error derivative
    double e = (c_s->r_ - e_s->p_);
    double de = (c_s->e_ - e) / dT;

    // Get PID output
    double ut = pid(e, de, pid_p);

    // Store for next cycle
    c_s->e_ = e;
    c_s->de_ = de;
    c_s->u_ = ut;
}

//--------------------------------------------------------------------------
void velocityControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p)
{
    // Sliding sum for encoder speed
    enc1->dp_ = enc1->dp_ * 0.99 + 0.01*(enc1->p_ - enc1->pp_)/(dT/TIME_SCALE);  // dT = 1000 us = 0.001 s
    enc1->pp_ = enc1->p_;

    //Set-point calculation
    double setPoint = minimumJerk(c_s->ti_, (double)t_new, c_s->T_, c_s->ri_, c_s->rf_);
    c_s->r_ = setPoint;

    // Error and error derivative
    double e = (c_s->r_ - e_s->dp_); 
    double de = (c_s->e_ - e) / dT;

    // Get PID output
    double ut = pid(e, de, pid_p);

    // Store for next cycle
    c_s->e_ = e;
    c_s->de_ = de;
    c_s->u_ = ut;
}


//--------------------------------------------------------------------------
float minimumJerk(float t0, float t, float T, float q0, float qf)
{
    // Calculate t / T. Clamp to [0,1] to prevent value explosion
    double tbyT = min((t-t0)/(T-t0),1);
    // Minimum jerk equation.
    return (q0 + (qf - q0) * (10.0 * pow(tbyT,3.0) - 15.0 * pow(tbyT,4.0) + 6.0 * pow(tbyT,5.0)));
}
//--------------------------------------------------------------------------
float pid(float e, float de, PIDParameters* p)
{
    // Update integral term
    p->I_ += e*dT;
    // Calculate output value
    double ut =  p->Kp_*e + p->Ki_ * p->I_ + p->Kd_ * de;
    // Clamp to maixmum and minimum value before returning
    ut = min(max(p->u_min_, ut), p->u_max_);
    return ut;
}

#if ROS_SUPPORT
//--------------------------------------------------------------------------
void updateState() {
    state.brake = digitalRead(motor1->BRK_);
    state.direction = digitalRead(motor1->DIR_);
    state.current = analogRead(motor1->CUR_);
    state.pwm = analogRead(motor1->PWM_);
    state.encoder = enc1->p_;
}

//////////////////////ROS Services ////////////////////////////////////
void setPosCallback(const arduino_pkg::SetPosition::Request &req, arduino_pkg::SetPosition::Response &res)
{
    // Start and end state of encoder
    mc1->rf_ = req.encoder; 
    mc1->ri_ = enc1->p_;
    mc1->r_ = enc1->p_;

    // Reset integral
    pid_mc1->I_ = 0;

    // Start and end times
    mc1->ti_ = micros();
    mc1->T_ = micros() + (abs(req.encoder - enc1->p_))/(V_MAX)*(15.0/8.0)*1e6;

    // Activate controller
    mc1->active_ = true;
    res.success = true;
}

void setPIDCallback(const arduino_pkg::SetPID::Request &req, arduino_pkg::SetPID::Response &res) {
    pid_mc1->Kd_ = req.k_d;
    pid_mc1->Kp_ = req.k_p;
    pid_mc1->Ki_ = req.k_i;
    res.success = true;    
}

void setVelCallback(const arduino_pkg::SetVelocity::Request &req, arduino_pkg::SetVelocity::Response &res)
{
    // Start and end state of encoder
    mc1->rf_ = req.ticksPerSecond;
    mc1->ri_ = enc1->dp_;
    mc1->r_ = enc1->dp_;

    // Reset integral
    pid_mc1->I_ = 0;

    // Start and end time
    mc1->ti_ = micros();
    // 0.006  = 3 s / 560 tps -- acceleration constant
    mc1->T_ = micros() + abs(mc1->ri_ - mc1->rf_) * 0.006*1e6;

    // Activate correct controller
    mc1->active_ = false;    
    res.success = true;
}

void setOn(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    mc1->on_ = true; 
}

void setOff(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    mc1->on_ = false;
}
#endif


void initMotor(MotorShieldPins* pins)
{
    pinMode(pins->DIR_,OUTPUT);
    pinMode(pins->BRK_,OUTPUT);
    pinMode(pins->PWM_,OUTPUT);
    pinMode(pins->CUR_,INPUT);

    digitalWrite(pins->DIR_,HIGH);
    digitalWrite(pins->BRK_,LOW);
}

////////////////////////////////////////////////////////////////
///////////////////////// MAIN LOOP ////////////////////////////
////////////////////////////////////////////////////////////////

void loop() {
#if ROS_SUPPORT
    //spin and check if we should publish
    t_new = micros();

    updateState();

    nh.spinOnce();
    if (abs(t_new - t_old_serial) > dT_serial) {
        updateState();
        state_publisher.publish(&state); r.data = mc1->r_;
        r_publisher.publish(&r); control.data = enc1->dp_;
        control_publisher.publish(&control); 
        t_old_serial = t_new;
    }
#endif

    //add the ROS overhead to the time since last loop
    t_new = micros();
    //Do nothing if the sampling period didn't pass yet
    if (abs(t_new - t_old) < dT)
        return;
    t_old = t_new;

    // Do nothing
    if (mc1->on_)
    {
        if (mc1->active_)
            positionControl(mc1, enc1, motor1, pid_mc1);
        else
            velocityControl(mc1, enc1, motor1, pid_mc1);
        actuate(mc1->u_, motor1);
    }

#if ROS_SUPPORT==0
    Serial.print("Encoder = ");
    Serial.println(enc1->p_);
#endif 

}
