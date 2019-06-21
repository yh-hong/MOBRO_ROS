/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic types you use.
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <time.h>
#include <iostream>
#include <algorithm>


#include <limits>

#include <tf/transform_listener.h>

using namespace cv;
using namespace std;

// You may have a number of globals here.

double resolution;

double x,y,theta;  //current position
int u,v; // image position;
geometry_msgs::Twist vel;

ros::Publisher scanPub;
ros::Publisher odomPub ;
ros::Publisher statePub ;


 
Mat house = cv::imread( "/home/yahui_hong/Documents/mobro_ros/maps/house.pgm", IMREAD_GRAYSCALE);
Mat house_draw = cv::imread( "/home/yahui_hong/Documents/mobro_ros/maps/house.pgm", IMREAD_COLOR);
Mat house_copy;
int nr = house.rows;
int nc = house.cols;

int u_shift ;
int v_shift ; 



// Callback functions...

const double pi  =3.141592653589793238463;
double inf = numeric_limits<double>::infinity();

void SetScan( ros::Time scan_time){



    //--------------------------------------------------------------
    //---------------- filling /scan ----------------------------------------- 

    // info about hokuyo 
    // https://www.hokuyo-aut.jp/search/single.php?serial=165

    // The LaserScan Message
    // http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors#Writing_Code_to_Publish_a_LaserScan_Message
    // https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/
    
    // Laser scans angles are measured counter clockwise, with 0 facing forward
    // (along the x-axis) of the device frame
    sensor_msgs::LaserScan scan;
    
    unsigned int num_readings = int(240/0.36);
    double laser_frequency = 10;

    scan.header.stamp = scan_time; 
    scan.header.frame_id = "/laser";
    scan.angle_min = -120*(pi/180); //angle correspond to FIRST beam in scan ( in rad)
    scan.angle_max = 120*(pi/180);  //angle correspond to LAST beam in scan ( in rad)
    scan.angle_increment = 0.36*(pi/180); // Angular resolution i.e angle between 2 beams 
    
    scan.time_increment = (1 / laser_frequency) / (num_readings);  //time between measurements [seconds]
    scan.range_min = 0.060; // minimum range value [m]
    scan.range_max = 4.095;  //  maximum range value [m]
    scan.ranges.resize(num_readings);

    //distance measure corresponds to angle -135 deg to 135 deg
    double alpha = scan.angle_min;
    for(unsigned int i = 0; i < num_readings; ++i){

        
        for (int j=0; ; j++){
            int ud,vd;

            ud = int(u + j*(cos(theta+alpha)));
            vd = int(v + j*(sin(theta+alpha)));
            circle(house_copy, Point2f( ud, vd), 2, Scalar( 0, 250, 250), 1, 8, 0);
           
            int value = house.at<uchar>(vd,ud);
            // int test1 = house.at<uchar>(116,56);
            // int test2 = house.at<uchar>(56,116);
            // cout<<"v1:"<<test1<<" v2:"<<test2<<endl;
            double dis = sqrt((u-ud)*(u-ud)+(v-vd)*(v-vd));

            // cout<<"new position: "<<ud<<" "<<vd<<endl;
            if(ud >=nc || ud<0 || vd >=nr || vd<0){
                scan.ranges[i]=inf;
                //cout<<"1";
                break;
                
            }
            else if(dis>int(scan.range_max/resolution) ){
                //cout<<"dis:" <<dis<< "compared: "<<int(scan.range_max/resolution)<<endl;
                scan.ranges[i]=inf;
                //cout<<"2";
                break;
                
            }            
            else if(value<100){
                scan.ranges[i]= dis;
                //cout<<"value:"<<value<<endl;
                //cout<<"distance"<<dis<<endl;
                circle(house_copy, Point2f( ud, vd), 2, Scalar( 0, 255, 0), 1, 8, 0);
                break;
                //cout<<"3"<<endl;
            }
                
               
            
        }

        
        alpha = alpha + scan.angle_increment;
        // ranges[0] = //distance measure corresponds to angle -135 deg
        // ranges[1080] = //distance measure corresponds to angle +135 deg
        // scan.ranges[i] = ranges[i];  // ranges[i] == ??
        // scan.intensities[i] = intensities[i]; // intensities == ??

    }
    
    scanPub.publish(scan);
    // ++count;
    

}




void velCallback(geometry_msgs::Twist velMessage)
{
    vel = velMessage;

}

// pub odom
// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "simulation_2d_node");

    // Define your node handles
    ros::NodeHandle nh("~");

    //--------------------------------------------------------------
    //-----------------------read yaml file and initialization------------------------------

    // initialization


    FileStorage fs("/home/yahui_hong/Documents/cpp_test/maps/house.yaml", FileStorage::READ);
    // read param
    if(!fs.isOpened()){
        cout<<"failed to read yaml file "<<endl;
        return 1;       
    }
    resolution = (double)fs["resolution"];
    vector<double> origin;

    fs["origin"]>>origin;    
    x = origin[0];
    y = origin[1];
    theta = origin[2]*(pi/180); 

    cout<<"resolution: "<<resolution<<endl;
    cout<<"origin: "<<x<<" "<<y<<" "<<theta<<endl;

    u_shift = int(x/resolution);
    v_shift = int(y/resolution);

    fs.release();

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;



    //------------------------------------------------------------------------------------------
    //-----------pub and sub-----------------------------------------------------------------------

    // Declare your node's subscriptions and service clients
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,velCallback );
    

    // Declare you publishers and service servers
    //statePub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
    scanPub = nh.advertise<sensor_msgs::LaserScan>("/scan",50);
    odomPub = nh.advertise<nav_msgs::Odometry>("/odom",50);
    tf::TransformBroadcaster odom_broadcaster;

   
    

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(10);   // Or other rate.

    ROS_INFO("ini finished");

    cout<<"nc:"<<nc<<" nr:"<<nr<<endl;

	while (ros::ok()){
		ros::spinOnce();
        house_draw.copyTo(house_copy);


        

        // ------------------------------publish odom-----------------------------------------

        current_time = ros::Time::now();

        double vx = vel.linear.x;
        
        double vth = vel.angular.z; 
        

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = vx * cos(-theta)  * dt;
        double delta_y = vx * sin(-theta) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        theta += delta_th;

         //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
   
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
   
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);


        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        

        odom.twist.twist.angular.z = vth;

        //publish the message
        odomPub.publish(odom);
        
        // draw odom
        u = int(x/resolution)-u_shift;
        v= nr-int(y/resolution)+v_shift-1;
        //v= nr-1-(int(y/resolution)-v_shift);
        //cout<<"u_shift:"<<u_shift<<"  v_shift:"<<v_shift<<endl;
        cout<<"x:"<<x<<" y:"<<y<<endl;
        cout<<"u:"<<u<<"  v:"<<v<<endl;

        //----------------publish joint state------------------------------------------

        

        // ----------------publish scan -----------------------------------------------        
        SetScan(current_time);
        arrowedLine(house_copy,Point2f(u,v), Point2f( u, v)+10*Point2f(cos(theta),sin(theta)),  Scalar( 255, 0, 0), 1, 8, 0, 0.1);

        imshow("house", house_copy);
        waitKey(1);
        


        // Your node's code goes here.
        last_time = current_time;

		rate.sleep();
    }
}

