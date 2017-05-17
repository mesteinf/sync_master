#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

const long double PI = 3.1415926535897932384;

//Exosystem / State - I.C. (now recieved from parameter server. Declaration in launch file)
double w1_init = 0, w2_init = 1, w3_init = 0, x_pos = 1.3, y_pos = -0.2, Heading = 1.5508;

//Initial Condition
double t = 0, time_04;

//Declare variable
double y[2] = {0, 0};
const double L = 0.2; // [m] - Distance to point in front of Robot to Control
double pathLength; //C = 2*pi*r = pi*d
double a = 1, b = 1, a0 = 0, b0 = 0; //Ellipse path data 
double xi;
double dxi[2] = {0, 0};
double deta[2] = {0, 0};
int gain = 5;
double error;
double angle = 0;
double det = 0;
double external_input[2] = {0, 0};
double control_input[2] = {0, 0};
double sigmaPrime_norm;
const double S[3][3] = {{0, 1, 0}, {0, 0, 0}, {0, 0, 0}};
const double Q[3] = {1, 0, 0};
double x1, x2, x3, w1, w2, w3;
double tmp1, tmp2, tmp3, tmp4;
const double loopRate = 100;
double dt = 1/loopRate; // time step
double w1_04 = -1, w2_04 = -1, w3_04 = -1;
double x1_04 = 0, x2_04 = 0, x3_04 = 0, v_04 = 0, angv_04 = 0;
double eta_Cham_03;
double WAVE4lin, WAVE4ang;

//Declare Functions
//Callback function for the Eta topic
void Exo_callback(const geometry_msgs::Point& msg) {

	w1_04 = msg.x;
	w2_04 = msg.y;
	w3_04 = msg.z;
}
//Callback function for the Position_04 topic
void Position_callback(const geometry_msgs::Twist& msg) {

	x1_04 = msg.linear.x;
	x2_04 = msg.linear.y;
	x3_04 = msg.linear.z;
	
	v_04 = msg.angular.x;
	angv_04 = msg.angular.y;
	time_04 = msg.angular.z;
	
}

int main(int argc, char **argv)
{
	
	//Initialize the ROS framework
    ros::init(argc,argv,"Agent_03");
    ros::NodeHandle n;
    
    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber eta_sub = n.subscribe("/Exo_04", 1, Exo_callback);
    ros::Subscriber pos_sub = n.subscribe("/position_04", 1, Position_callback);
    //Setup topics to Publish from this node
    //ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); 
   ros::Publisher eta_pub = n.advertise<geometry_msgs::Point>("/Exo_03", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;
    geometry_msgs::Point Exo_03;
    
    //Store Time via ROS to print to file
    ros::Time currentTime;
    	
    //Set the loop rate
    ros::Rate loop_rate(loopRate);    //20Hz update rate

	//States
    x1 = x_pos;
    x2 = y_pos;
    x3 = Heading;
    
    w1 = w1_init;
    w2 = w2_init;
    w3 = w3_init;
    
    pathLength = 2*PI*a;
    dt = 0.001;
    
    time_t rawTime;
    char fileName[80];
    time(&rawTime);
    sprintf(fileName, "/home/turtlebot3/catkin_ws/src/unicycle_sync_master/bin/%s.txt",ctime(&rawTime));
    
    char *p = fileName;
    for (; *p; ++p) {
    	
    	if (*p == ' ' || *p == ':')
    		*p = '_';
    }
    
    FILE *fptr;
    fptr = fopen(fileName, "w");
    fprintf(fptr, "Agent03, X-pos03, Y-pos03, Heading03, Velocity03, Angular Velocity03, Exostate_103, Exostate_203, Exostate_303, Discrete_Time03, ROS Time03, Agent04, X-pos04, Y-pos04, Heading04, Velocity04, Angular Velocity04, Exostate_104, Exostate_204, Exostate_304, Discrete_Time04\n");
    
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
		    vel.linear.x = 2;
		    vel.angular.z = 2;
		    vel_pub.publish(vel);
    	//Main loop code goes 	
	if (w2_04 >= 0) {

		    y[0] = x1 + L*cos(x3);
		    y[1] = x2 + L*sin(x3);
		    
		    // Co-ordinate Transformation
		    // s(y) = (y1-a0)^2/a^2 + (y2-b0)^2/b^2 - 1
		    //a = pathData[0]; b = pathData[1]; a0 = pathData[2]; b0 = pathData[3];		Now assigned using paramServer on ROS with launch file
		    
		    // tangential states
		    xi = pow(((y[0] - a0)/a),2) + pow(((y[1] - b0)/b),2) - 1;
		    dxi[0] = 2*(y[0] - a0)/pow(a,2);
		    dxi[1] = 2*(y[1] - b0)/pow(b,2);
		    
		    //fprintf(fptr,"Xi: %f\n dxi: %f, %f\n",xi,dxi[0],dxi[1]);
		    
		    // transversal states
		    //For the case of a circle only integral(f_anon) = R*lambdaStar
		    //arc length
		    eta_Cham_03 = atan2((y[1] - b0), (y[0] - a0)); // angle [-pi, pi)
		    if (eta_Cham_03 < 0) 							//angle [0, 2*pi)
		    	eta_Cham_03 = 2*PI + eta_Cham_03;
		     sigmaPrime_norm = sqrt(pow(b*cos(eta_Cham_03 ),2) + pow(-a*sin(eta_Cham_03 ),2));
		     //fprintf(fptr,"LambdaStar: %f\n",lambdaStar);
		    deta[0] = -(a*sin(eta_Cham_03))/sigmaPrime_norm;
		    deta[1] =  (b*cos(eta_Cham_03))/sigmaPrime_norm;
		    eta_Cham_03 = a*eta_Cham_03; // arc-length [0, pathLength)
		    
		    error = eta_Cham_03 - (pathLength/(2*PI))*fmod((Q[0]*w1 + Q[1]*w2 + Q[2]*w3),2*PI);
		    // MATLAB angle(exp(1j*(2*pi/pathLength)*error_A))
		    angle = atan2(sin((2*PI/pathLength)*error), cos((2*PI/pathLength)*error));
		    
		    external_input[0] = -gain*xi;
		    external_input[1] = -gain*angle + (pathLength/(2*PI))*(w1*(Q[0]*S[0][0] + Q[1]*S[1][0] + Q[2]*S[2][0]) + w2*(Q[0]*S[0][1] + Q[1]*S[1][1] + Q[2]*S[2][1]) + w3*(Q[0]*S[0][2] + Q[1]*S[1][2] + Q[2]*S[2][2]));
		    //MATLAB [cos(x3), sin(x3); -sin(x3)/L, cos(x3)/L]*([dh1_A;dh2_A]\(external_input_A));
		    det = dxi[0]*deta[1] - dxi[1]*deta[0];    	    
		    
		    tmp1 = (1/det)*(external_input[0]*deta[1] + external_input[1]*-dxi[1]);
		    tmp2 = (1/det)*(external_input[0]*-deta[0] + external_input[1]*dxi[0]);
		    
		    control_input[0] = tmp1*cos(x3) + tmp2*sin(x3);
		    control_input[1] = tmp1*-sin(x3)/L + tmp2*cos(x3)/L;

		    currentTime = ros::Time::now();
		    fprintf(fptr, "Agent_03, %f , %f , %f, %f, %f, %f, %f, %f, %f, %f, 04, %f , %f , %f, %f, %f, %f, %f, %f, %f\n", x1, x2, x3, control_input[0], control_input[1], w1, w2, w3, t, currentTime.toSec(), x1_04, x2_04, x3_04, v_04, angv_04, w1_04, w2_04, w3_04, time_04);
		    
		    vel.linear.x = control_input[0];
		    vel.angular.z = control_input[1];
		    vel_pub.publish(vel);	
			
		    Exo_03.x = w1;
		    Exo_03.y = w2;
		    Exo_03.z = w3;
		    eta_pub.publish(Exo_03);
		    
		    //State Dynamics
		    x1 = x1 + cos(x3)*control_input[0]*dt;
		    x2 = x2 + sin(x3)*control_input[0]*dt;
		    x3 = x3 + control_input[1]*dt;
		    
		    //State Dynamics
		    w1 = w1 + ((S[0][0]*w1 + S[0][1]*w2 + S[0][2]*w3) + (w1_04 - w1))*dt;
		    w2 = w2 + ((S[1][0]*w1 + S[1][1]*w2 + S[1][2]*w3) + (w2_04 - w2))*dt;
		    w3 = w3 + ((S[2][0]*w1 + S[2][1]*w2 + S[2][2]*w3) + (w3_04 - w3))*dt;
		    
		    t += dt;
		    }	
	}
    return 0;
}
