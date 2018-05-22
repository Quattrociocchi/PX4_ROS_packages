#include <ros/ros.h>
#include <Eigen/Dense>

#include "utils.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Quaternion.h>

#include <qcontrol_defs/PosControl.h>
#include <qcontrol_defs/VelControl.h>
#include <qcontrol_defs/AccControl.h>
#include <qcontrol_defs/AttControl.h>
#include <qcontrol_defs/PVA.h>

#include <qcontrol_defs/CommandAction.h>
#include <qcontrol_defs/QuadState.h>

//#define QUAD_VICON_TOPIC                                "/vicon/"
#define COMMAND_TOPIC                                   "qcontrol/commands"
#define ATT_CONTROL_TOPIC                               "qcontrol/att_control"
#define POS_CONTROL_TOPIC                               "qcontrol/pos_control"
#define ACC_CONTROL_TOPIC                               "qcontrol/acc_control"
#define VEL_CONTROL_TOPIC                               "qcontrol/vel_control"
#define PVA_CONTROL_TOPIC                               "qcontrol/pva_control"
#define OFFBOARD_INFO                                   "qcontrol/offboard_info"

#define POS_VEL_CONTROL                 3
#define ACC_ATT_CONTROL                 5     

#define MAIN_LOOP_RATE             		40            		//main loop rate for getting more faster all the subscriber datas

#define PI_                   			3.1415926535		//3.1415926535897
#define TIME_CONSTANT                   0.2

#define PITCH_MAX               		(PI_/10.0)    		//Max pitch angle allowed
#define ROLL_MAX                		(PI_/10.0)    		//Max roll angle allowed
#define YAW_MAX_RATE                    (PI_/5.0)           //in deg per seg

#define THROTTLE_MAX             		1.0           		//Max throttle allowed

#define STEP_Z_RATE                     0.05
#define STEP_Y_X_RATE                   0.05
#define STEP_ROLL_PITCH                 (PI_/90.0)          //increase the roll and yaw max value by this at evry key pressed
#define STEP_YAW_RATE                   (PI_/40.0)          //increase max yaw_rate by this
//Joystick configuration

#define THROTTLE_AXE            		1            		//Up/Down left axis
#define BODY_WORLD                      0                   //Left/Right left axis
#define YAW_LEFT_AXE            		2            		//LT
#define YAW_RIGHT_AXE            		5            		//RT
#define PITCH_AXE                		4            		//Up/down right axis          
#define ROLL_AXE                		3            		//left/right right axis

#define POS_ATT		                   	5					//RB
#define VEL_ACC                         4                   //LB

#define ARM_MOTOR                		7            		//Start button
#define PVA_BUTTON   					6					//back button
#define POS_VEL 						2					//X button
#define ATT_ACC 						0					//A button
#define TAKEOFF							1					//B button
#define LAND							3					//Y button

#define STEP_Z_INCR                		13            		//cross key up
#define STEP_Z_DECR                		14            		//cross key down
#define STEP_LEFT_RIGHT_INCR    		12            		//cross key right
#define STEP_LEFT_RIGHT_DECR    		11            		//cross key left 	   


//Publisher for differents kind of quad control
ros::Publisher pos_control_pub;
ros::Publisher att_control_pub;
ros::Publisher vel_control_pub;
ros::Publisher acc_control_pub;
ros::Publisher pva_control_pub;

ros::ServiceClient command_client;                      //Command service for request to the main node


//Axes mapping for moving around the quad
int left_axe_up_down;                          //Up|down left axis
int left_axe_left_right;                       //Left/Right left axis
int yaw_left_axe;                              //LT joystick
int yaw_right_axe;                             //Rt joystick
int right_axe_up_down;                         //Up|down right axes
int right_axe_left_right;                      //Left|right right axes 

//Button action to trigger control state changes
int LB;                                        //Lb button
int RB;                                        //Rb button

int B;                                         //B button
int Y;                                         //Y button

int X;                                         //X button
int A;                                         //A button

int vertical_rate_incr;                        //cross key up
int vertical_rate_decr;                        //cros key down
int horizontal_rate_incr;                      //cross key right
int horizontal_rate_decr;                      //cross key left

int start;                                     //Start button
int back;                                      //Back button

double time_constant;                                   //Time constant for the filtering

//attitude control max values
double pitch_max = PITCH_MAX;
double roll_max = ROLL_MAX;
double yaw_max_rate = YAW_MAX_RATE;
double throttle_max = THROTTLE_MAX;

//velocity control step for  x,y and z axis
double max_speed_z = 1.6;
double max_speed_y = 1.3;
double max_speed_x = 1.3;

//Main values to send like position, velocity and acceleration
qcontrol_defs::PosControl pos_control_msg;
qcontrol_defs::AccControl acc_control_msg;
qcontrol_defs::VelControl vel_control_msg;
qcontrol_defs::AttControl att_control_msg;
qcontrol_defs::PVA pva_control_msg;

//Informations on the current state of the quad
qcontrol_defs::QuadState quad_state;

//Getting feedback from the fcu
nav_msgs::Odometry current_odom;
double mc_given_yaw;

double current_yaw;
geometry_msgs::Point current_pos;
geometry_msgs::Vector3 current_vel;
geometry_msgs::Vector3 current_acc;

//store current control state of the quad
unsigned char current_state = -1;

//save last ARM button press status
bool last_is_arm = false;

//Store axes moves of the joystick
double up_down(0) ,  left_right(0) , back_forward(0) , yaw(0);
bool is_body_frame = false;

//Specify with robot is actually controlled
bool is_mc  = true;

//From velocity , filter to get PVA
void filterJoy(qcontrol_defs::PVA &PVA_ref,
                        const double vx , const double vy , const double vz,
                        double dt){

    const double zeta = 1.0;                    //Critically damped
    const double wn = 1.0/time_constant;        //Time constant = 1/(zeta.wn)
    Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity(3,3);

    //Second order low pass filter continuous model
    Eigen::Matrix3d A;
    Eigen::Vector3d B;

    A << 0,      1,          0,
         0,      0,          1,
         0, -wn*wn, -2*zeta*wn;
    B << 0,
         0,
         wn*wn;

    //Current states in x,y,z direction
    Eigen::Vector3d Xk_x, Xk_y, Xk_z;
    Xk_x << PVA_ref.pos.x,
            PVA_ref.vel.x,
            PVA_ref.acc.x;
    Xk_y << PVA_ref.pos.y,
            PVA_ref.vel.y,
            PVA_ref.acc.y;
    Xk_z << PVA_ref.pos.z,
            PVA_ref.vel.z,
            PVA_ref.acc.z;

    //Propagate states (crude euler integration)
    Eigen::Vector3d Xk1_x, Xk1_y, Xk1_z;
    Xk1_x = (I_3x3 + A*dt)*Xk_x + dt*B*vx;
    Xk1_y = (I_3x3 + A*dt)*Xk_y + dt*B*vy;
    Xk1_z = (I_3x3 + A*dt)*Xk_z + dt*B*vz;

    //Update PVA structure
    PVA_ref.pos.x = Xk1_x[0];
    PVA_ref.pos.y = Xk1_y[0];
    PVA_ref.pos.z = Xk1_z[0];

    PVA_ref.vel.x = Xk1_x[1];
    PVA_ref.vel.y = Xk1_y[1];
    PVA_ref.vel.z = Xk1_z[1];

    PVA_ref.acc.x = Xk1_x[2];
    PVA_ref.acc.y = Xk1_y[2];
    PVA_ref.acc.z = Xk1_z[2];
}

//Main joy callback function for dealing with Joystick event
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){

    //Commands message to send to the service
    bool send_command = false;
    qcontrol_defs::CommandAction commands;

    //Arm or disarm motors
    if (msg->buttons[start] == 1 && !last_is_arm){
        commands.request.arm_motors = !quad_state.is_armed ? 1: 2;  //Toggle arming status
        send_command = true;
    }

    //Take off started
    if (msg->buttons[B] == 1){
        commands.request.start_takeoff = 1;
        send_command = true;
    }

    //landing started
    if (msg->buttons[Y] == 1){
        commands.request.start_landing = 1;
        send_command = true; 
    }

    //PVA control mode enabled
    if (msg->buttons[back] == 1){
        commands.request.is_pvactl = 1;
        send_command = true;
    }

    //Enable attitude or acceleration control
    if (msg->buttons[A] == 1){
        current_state = ACC_ATT_CONTROL;
    }

    //Enable position or velocity control
    if (msg->buttons[X] == 1){
        current_state = POS_VEL_CONTROL;
    }

    //Enable position control
    if(current_state == POS_VEL_CONTROL && msg->buttons[RB] ==1){
        commands.request.is_posctl = 1;
        send_command = true;
    }

    //Enable Velocity control
    if(current_state == POS_VEL_CONTROL && msg->buttons[LB] ==1){
        commands.request.is_velctl = 1;
        send_command = true;
    }

    //Enable attitude control
    if(current_state == ACC_ATT_CONTROL && msg->buttons[RB] == 1){
        commands.request.is_attctl = 1;
        send_command = true;
    }

    //Enable acceleration control
    if(current_state == ACC_ATT_CONTROL && msg->buttons[LB] == 1){
        commands.request.is_accctl = 1;
        send_command = true;
    }

    //Make a call to the service if a type of control is requested
    if(send_command){
        current_yaw = mc_given_yaw;
        current_pos = current_odom.pose.pose.position;
        current_vel = current_odom.twist.twist.linear;
        current_acc = geometry_msgs::Vector3();
        command_client.call(commands);
    }

    if(msg->axes[left_axe_left_right] > 0.7){
        ROS_INFO ("BODY FRAME ENABLED");
        is_body_frame = true;
    }

    if(msg->axes[left_axe_left_right] < -0.7){
        ROS_INFO ("BODY FRAME DiSABLED");
        is_body_frame = false;
    }

    //Saving moves command
    up_down = msg->axes[left_axe_up_down];
    left_right = msg->axes[right_axe_left_right];
    back_forward = msg->axes[right_axe_up_down];
    yaw = -((msg->axes[yaw_left_axe] - 1.0)/2.0) + ((msg->axes[yaw_right_axe] - 1.0)/2.0);

    if ( msg->buttons[vertical_rate_incr]==1 || msg->buttons[vertical_rate_decr]==1 || msg->buttons[horizontal_rate_incr]==1
                                    || msg->buttons[horizontal_rate_decr]==1){
        if(quad_state.is_attctl){
            yaw_max_rate += msg->buttons[vertical_rate_incr]*STEP_YAW_RATE - msg->buttons[vertical_rate_decr]*STEP_YAW_RATE;
            pitch_max += msg->buttons[horizontal_rate_incr]*STEP_ROLL_PITCH - msg->buttons[horizontal_rate_decr]*STEP_ROLL_PITCH;
            roll_max  += msg->buttons[horizontal_rate_incr]*STEP_ROLL_PITCH - msg->buttons[horizontal_rate_decr]*STEP_ROLL_PITCH;
            ROS_WARN("[ATTITUDE CONTROL] CURRENT pitch_max , roll_max , yaw_rate_max : %f   , %f   , %f ", pitch_max , roll_max , yaw_max_rate);
        }else if(quad_state.is_velctl || quad_state.is_posctl || quad_state.is_pvactl || quad_state.is_accctl){
            max_speed_z += msg->buttons[vertical_rate_incr]*STEP_Z_RATE - msg->buttons[vertical_rate_decr]*STEP_Z_RATE;
            max_speed_y += msg->buttons[horizontal_rate_incr]*STEP_Y_X_RATE - msg->buttons[horizontal_rate_decr]*STEP_Y_X_RATE;
            max_speed_x += msg->buttons[horizontal_rate_incr]*STEP_Y_X_RATE - msg->buttons[horizontal_rate_decr]*STEP_Y_X_RATE;
            ROS_WARN("[SPEED RATES] CURRENT vz_max , vy_max , vx_max : %f  , %f  , %f ", max_speed_z , max_speed_y , max_speed_x);
        }
    }

    //Resetting is_arm variable for being able to toggle between armed and disarmed 
    last_is_arm = (msg->buttons[start] == 1);

}

//Odometry data callback
void curr_pos_callback(const nav_msgs::Odometry::ConstPtr& msg){
    current_odom = *msg;
    mc_given_yaw = getHeadingFromQuat(msg->pose.pose.orientation);
}

//Current state of the quadcopter from the offboard controller
void current_state_callback(const qcontrol_defs::QuadState::ConstPtr& msg){
	quad_state = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"joy_command_node");
    ros::NodeHandle nh_params("~");

    if(!nh_params.getParam("time_constant" , time_constant)){
        time_constant = TIME_CONSTANT;
        ROS_WARN("No parameter time_constant provided. Using default value %f !",time_constant); 
    }
    if(!nh_params.getParam("LB",LB)){
        LB = VEL_ACC;
        ROS_WARN("No parameter LB provided. Using default value %d !",LB);
    }

    if(!nh_params.getParam("RB",RB)){
        RB = POS_ATT;
        ROS_WARN("No parameter RB provided. Using default value %d !",RB);
    }

    if(!nh_params.getParam("LT",yaw_left_axe)){
        yaw_left_axe = YAW_LEFT_AXE;
        ROS_WARN("No parameter LT provided. Using default value %d !",yaw_left_axe);
    }

    if(!nh_params.getParam("RT",yaw_right_axe)){
        yaw_right_axe = YAW_RIGHT_AXE;
        ROS_WARN("No parameter RT provided. Using default value %d !", yaw_right_axe);
    }

    if(!nh_params.getParam("Start",start)){
        start = ARM_MOTOR;
        ROS_WARN("No parameter Start provided. Using default value %d !",start);
    }

    if(!nh_params.getParam("Back",back)){
        back = PVA_BUTTON;
        ROS_WARN("No parameter Back provided. Using default value %d !",back);
    }

    if(!nh_params.getParam("X", X)){
        X = POS_VEL;
        ROS_WARN("No parameter X provided. Using default value %d !",X);
    }

    if(!nh_params.getParam("Y" , Y)){
        Y = LAND;
        ROS_WARN("No parameter Y provided. Using default value %d !",Y);
    }

    if(!nh_params.getParam("A" , A)){
        A = ATT_ACC;
        ROS_WARN("No parameter A provided. Using default value %d !",A);
    }

    if(!nh_params.getParam("B" , B)){
        B = TAKEOFF;
        ROS_WARN("No parameter B provided. Using default value %d !", B);
    }

    if(!nh_params.getParam("up_down_axis_left" , left_axe_up_down)){
        left_axe_up_down = THROTTLE_AXE;
        ROS_WARN("No parameter up_down_axis_left provided. Using default value %d !",left_axe_up_down);
    }

    if(!nh_params.getParam("left_right_axis_left" , left_axe_left_right)){
        left_axe_left_right = BODY_WORLD;
        ROS_WARN("No parameter left_right_axis_left provided. Using default value %d !",left_axe_left_right);
    }

    if(!nh_params.getParam("left_right_axis_right" , right_axe_left_right)){
        right_axe_left_right = ROLL_AXE;
        ROS_WARN("No parameter left_right_axis_right provided. Using default value %d !",right_axe_left_right);
    }

    if(!nh_params.getParam("up_down_axis_right" , right_axe_up_down)){
        right_axe_up_down = PITCH_AXE;
        ROS_WARN("No parameter up_down_axis_right provided. Using default value %d !",right_axe_up_down);
    }

    if(!nh_params.getParam("left_cross_key", horizontal_rate_decr)){
        horizontal_rate_decr = STEP_LEFT_RIGHT_DECR;
        ROS_WARN("No parameter left_cross_key provided. Using default value %d !",horizontal_rate_decr);
    }

    if(!nh_params.getParam("right_cross_key" , horizontal_rate_incr)){
        horizontal_rate_incr = STEP_LEFT_RIGHT_INCR;
        ROS_WARN("No parameter right_cross_key provided. Using default value %d !",horizontal_rate_incr);
    }

    if(!nh_params.getParam("down_cross_key" , vertical_rate_decr)){
        vertical_rate_decr = STEP_Z_DECR;
        ROS_WARN("No parameter down_cross_key provided. Using default value %d !",vertical_rate_decr);    
    }

    if(!nh_params.getParam("up_cross_key" , vertical_rate_incr)){
        vertical_rate_incr = STEP_Z_INCR;
        ROS_WARN("No parameter up_cross_key provided. Using default value %d !",vertical_rate_incr); 
    }

    if(!nh_params.getParam("throttle_max" , throttle_max)){
        throttle_max = THROTTLE_MAX;
        ROS_WARN("No parameter throttle_max provided. Using default value %f deg!",throttle_max); 
    }

    if(!nh_params.getParam("yaw_rate_max" , yaw_max_rate)){
        yaw_max_rate = YAW_MAX_RATE;
        ROS_WARN("No parameter yaw_rate_max provided. Using default value %f deg!",180.0*yaw_max_rate/PI_); 
    }

    if(!nh_params.getParam("pitch_max" , pitch_max)){
        pitch_max = PITCH_MAX;
        ROS_WARN("No parameter pitch_max provided. Using default value %f deg!",180.0 * pitch_max/PI_); 
    }

    if(!nh_params.getParam("roll_max" , roll_max)){
        roll_max = ROLL_MAX;
        ROS_WARN("No parameter roll_max provided. Using default value %f deg!",180.0 * roll_max /PI_); 
    }

    if(!nh_params.getParam("is_multicopter" , is_mc)){
        is_mc = true;
        ROS_WARN("No parameter is_multicopter provided. Using default value %d !",is_mc); 
    }

    bool only_command;
    if(!nh_params.getParam("only_command" , only_command)){
        only_command = false;
        ROS_WARN("No parameter only_command provided. Using default value %d !",only_command); 
    }

    /*std::string quad_name;
    if(!nh_params.getParam("quad_name",quad_name)){
        quad_name = "Quad8";
        ROS_WARN("No quad name provided. Default quad name is %s",quad_name.c_str());
    }*/

    //Appropriate node_handle
    ros::NodeHandle nh = ros::NodeHandle();
    //Publishers
    pos_control_pub = nh.advertise<qcontrol_defs::PosControl>(POS_CONTROL_TOPIC , 10);
    att_control_pub = nh.advertise<qcontrol_defs::AttControl>(ATT_CONTROL_TOPIC , 10);
    vel_control_pub = nh.advertise<qcontrol_defs::VelControl>(VEL_CONTROL_TOPIC , 10);
    acc_control_pub = nh.advertise<qcontrol_defs::AccControl>(ACC_CONTROL_TOPIC , 10);
    pva_control_pub = nh.advertise<qcontrol_defs::PVA>(PVA_CONTROL_TOPIC , 10);

    //Subscribers
    ros::Subscriber joy_sub =nh.subscribe<sensor_msgs::Joy>("joy",10,joy_callback);
    ros::Subscriber odom_subscriber = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom",10,curr_pos_callback);
    ros::Subscriber current_state = nh.subscribe<qcontrol_defs::QuadState>(OFFBOARD_INFO,10,current_state_callback);

    //Service
    command_client= nh.serviceClient<qcontrol_defs::CommandAction>(COMMAND_TOPIC);

    //Main loop rate
    ros::Rate main_rate(MAIN_LOOP_RATE);

    for (int i=0; i<MAIN_LOOP_RATE;i++){
    	ros::spinOnce();
    	main_rate.sleep();
    }

    //setting initial yaw and initial position
	current_yaw = mc_given_yaw;
	current_pos = current_odom.pose.pose.position;
    current_vel = current_odom.twist.twist.linear;

	ROS_WARN("[JOY NODE ]:  initial yaw : %f ",current_yaw);

    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration dt;

    while(ros::ok()){

        current_time = ros::Time::now();
        dt = current_time - last_time;
        last_time = current_time;

        if(!only_command && quad_state.is_posctl){       //Position control
            current_pos.x += back_forward * max_speed_x * dt.toSec();
            current_pos.y += left_right * max_speed_y * dt.toSec();
            current_pos.z += up_down * max_speed_z * dt.toSec();
            current_yaw += yaw * (yaw_max_rate) * dt.toSec();
            pos_control_msg.yaw = current_yaw;
            pos_control_msg.pos = current_pos;
            pos_control_pub.publish(pos_control_msg);
        }
        if(!only_command && quad_state.is_velctl){   //Velocity control state
            vel_control_msg.vel.x = back_forward * max_speed_x;
            vel_control_msg.vel.y = left_right * max_speed_y;
            vel_control_msg.vel.z = up_down * max_speed_z;
            vel_control_msg.yaw_rate = yaw * (yaw_max_rate);
            vel_control_msg.is_body_frame = is_body_frame;
            vel_control_pub.publish(vel_control_msg);
        }
        if(!only_command && quad_state.is_accctl){   //Acceleration control state
            acc_control_msg.acc.x = back_forward * max_speed_x; //Theorically incorrect 
            acc_control_msg.acc.y = left_right * max_speed_y;   //Theorically incorrect
            acc_control_msg.acc.z = up_down * max_speed_z;      //Theorically incorrect
            acc_control_msg.is_body_frame = is_body_frame;
            acc_control_pub.publish(acc_control_msg);
        }
        if(!only_command && quad_state.is_attctl){       //Attitude control
            if (is_mc){
                att_control_msg.roll = - left_right * roll_max;
                att_control_msg.pitch = back_forward * pitch_max;
                current_yaw += yaw * (yaw_max_rate) * dt.toSec();
            } else{
                att_control_msg.roll = 0;
                att_control_msg.pitch = 0;
                current_yaw += left_right * (yaw_max_rate) * dt.toSec();
            }
            att_control_msg.thrust = up_down * throttle_max ;
            att_control_msg.yaw = current_yaw;
            att_control_pub.publish(att_control_msg);
        }
        if(!only_command && quad_state.is_pvactl){       //PVA control
            pva_control_msg.pos = current_pos;
            //pva_control_msg.vel = geometry_msgs::Vector3();
            //pva_control_msg.acc = geometry_msgs::Vector3();
            pva_control_msg.vel = current_vel;
            filterJoy(pva_control_msg, back_forward * max_speed_x , left_right * max_speed_y , up_down * max_speed_z , dt.toSec());
            //std::cout << current_pos << "\n" << std::endl;
            //std::cout << current_vel << "\n" << std::endl; 
            //std::cout << current_acc << "\n" << std::endl; 
            current_pos = pva_control_msg.pos;
            current_vel = pva_control_msg.vel;
            current_acc = pva_control_msg.acc;
            current_yaw += yaw * (yaw_max_rate) * dt.toSec();
            pva_control_msg.yaw = current_yaw;
            pva_control_msg.is_body_frame = is_body_frame;
            pva_control_pub.publish(pva_control_msg);
        }

    	ros::spinOnce();
    	main_rate.sleep();
    }

    return 0;
}