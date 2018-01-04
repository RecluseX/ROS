
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/transform_broadcaster.h"
#include<tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_broadcaster.h"
#include "MCUBridge.h"
#include "SensorInterface.h"
#include "BenebotTypeDefine.h"

#include "BaseTimeApi.h"
#define DEG2RAD(x) x*M_PI/180.0
bool gResetFlag = false;
//define function  

ros::Time ToROSTime(S32 timeTick)
{
    return ros::Time((S32)timeTick/1000,long(timeTick%1000)*1000000);
}
float  RESOLUTION  = 1440.0/360.0 ;
float _maskDeg_[3][2] = {0};

void ldsDataFilter(const std::vector<float> pilarRange , float ldsOffset)
{
	if(pilarRange.size() != 6 )
		return;
	_maskDeg_[0][0] = (pilarRange[0] +  ldsOffset)*RESOLUTION;
	_maskDeg_[0][1] = (pilarRange[1]  +  ldsOffset)*RESOLUTION;
	_maskDeg_[1][0] = (pilarRange[2] +  ldsOffset)*RESOLUTION;
	_maskDeg_[1][1] = (pilarRange[3] +  ldsOffset)*RESOLUTION;

	_maskDeg_[2][0] = (pilarRange[4] +  ldsOffset)*RESOLUTION;
	_maskDeg_[2][1] = (pilarRange[5] +   ldsOffset)*RESOLUTION;
	return ;
}


class Node{

    public:
        Node();
        ~Node();
        bool initialize();
        void spinForever();
    private:
        string mcu_comm_name_;
        string lds_comm_name_;
        Odom_Type lastReading_;
        float xx_;
        float yy_;
        float theta_;
        
        ros::Publisher odom_pub_;
        ros::Publisher scan_pub_;
        ros::Subscriber cmd_sub_;
        ros::NodeHandle node_handle_;
        ros::NodeHandle private_nh_;
        vector<ros::WallTimer> wallTimers_;
        tf2_ros::TransformBroadcaster odom_broadcaster_;
        MCUBridge* mcu_bridge_;
        SensorInterfaceAPI* lds_bridge_;

        //function define
        void PublishOdom(const ros::WallTimerEvent& event);
        void PublishScan(const ros::WallTimerEvent& event);
        void HandleVelCMD(const geometry_msgs::TwistConstPtr&  vel_cmd);



};
Node::Node():private_nh_("~")
{
   private_nh_.param("/base_move_node/mcu_comm",mcu_comm_name_,std::string("/dev/ttyS0"));
   private_nh_.param("/base_move_node/lds_comm",lds_comm_name_,std::string("/dev/ttyS1"));

   std::vector<float> pillar_angle;
   if(private_nh_.getParam("/pillar_angle",pillar_angle))
       ROS_INFO("success gt pillar angle: %d",pillar_angle.size());
   else ROS_INFO("fail to get pillar angle:%d",pillar_angle.size());
   int lds_offset = 0 ;
   ldsDataFilter(pillar_angle,lds_offset);
   lastReading_.LDist = 0;
   lastReading_.RDist = 0;
   lastReading_.Gyro = 0;

   xx_=yy_=theta_=0.0;

   

}
Node::~Node()
{
    if(mcu_bridge_ != NULL)
    {
       delete mcu_bridge_; 
    }
    if(lds_bridge_ != NULL)
    {
        delete lds_bridge_;
    }
}
bool Node::initialize()
{
    mcu_bridge_ = new MCUBridge;
    ROS_INFO("mcu_comm_name_:%s",mcu_comm_name_.c_str());
    int rstFlag = 0;
    if(mcu_bridge_->ApplicationMCUStart(mcu_comm_name_) !=0 )
    {
        ROS_WARN("please check your MCU port!");

    }
    else{
        rstFlag += 1;
       usleep(100);
      mcu_bridge_->GYIODistanceReset(0);
      
      gResetFlag = true;
      wallTimers_.push_back(node_handle_.createWallTimer(ros::WallDuration(0.02),&Node::PublishOdom,this));
      odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>("/world",10);
      cmd_sub_ = node_handle_.subscribe("/cmd_vel",1,&Node::HandleVelCMD,this);

    }
/*    ROS_INFO("lds_comm_name_:%s",lds_comm_name_.c_str());
    lds_bridge_ = new SensorInterfaceAPI;
    if( lds_bridge_->SensorInterfaceInit(lds_comm_name_))
    {
        ROS_WARN("please check your lds port!");
    }
    else{
      rstFlag += 1;
      usleep(100);
      scan_pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan",1);
      wallTimers_.push_back(node_handle_.createWallTimer(ros::WallDuration(0.06),&Node::PublishScan,this));

    }*/
    if(rstFlag > 0)
    return true;
    else return false;
}
void Node::HandleVelCMD(const geometry_msgs::TwistConstPtr&  vel_cmd)
{
    geometry_msgs::Twist msg = *vel_cmd;
    double vx = msg.linear.x;
    double wz = msg.angular.z;
    double dl = 0.282;
    S16 vr = floor(((wz*dl + 2.0*vx)/2.0)*1000);
    S16 vl = floor((2.0*vx - vr/1000.0 )*1000);
    ROS_INFO("vr:%d,vl:%d,vx:%f,wz:%f",vr,vl,vx,wz);
    if(mcu_bridge_ != NULL)
    {
        mcu_bridge_->NavyMCUMoveCtrl(vl,vr,600,600);
    }
    return;
}
void Node::PublishOdom(const ros::WallTimerEvent& event)
{
    if(mcu_bridge_ == NULL)
        return;
    Odom_Type rawOdom;
    if(mcu_bridge_->GetOdomData(&rawOdom) == false)
        return;
/*
    short s16Deg = 0;
    char s8Flag =0;
    if(mcu_bridge_->NavyMCUMegneticGetData(&s16Deg,&s8Flag) == 0)
    {
        ROS_INFO("received megnetic data:%d",s16Deg);
    }
    else
        ROS_WARN("there is no megnetic received");*/

    if(gResetFlag == true)
    {
	lastReading_.LDist = rawOdom.LDist;
	lastReading_.RDist = rawOdom.RDist;
	lastReading_.Gyro = rawOdom.Gyro;
        gResetFlag = false;
        return;
    }
    float dL = float(rawOdom.LDist - lastReading_.LDist)/1000.0;
    float dR = float(rawOdom.RDist - lastReading_.RDist)/1000.0;

    double deltaYaw = double(rawOdom.Gyro - lastReading_.Gyro)/100.0;
     deltaYaw =-1*DEG2RAD(deltaYaw);

     float deltaS = (dL+dR)/2.0;
     float deltaX = deltaS*std::cos(deltaYaw);
     float deltaY = deltaS*std::sin(deltaYaw);

     float dx_world = std::cos(theta_)*deltaX - std::sin(theta_)*deltaY;
     float dy_world = std::sin(theta_)*deltaX + std::cos(theta_)*deltaY;

     xx_ += dx_world; yy_ += dy_world; theta_ += deltaYaw;
     

     //publish tf
     geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(theta_);
     geometry_msgs::TransformStamped odom_trans;
     odom_trans.header.stamp = ToROSTime(rawOdom.timeStamp);
     odom_trans.header.frame_id = "world";
     odom_trans.child_frame_id = "base_link";
     odom_trans.transform.translation.x = xx_;
     odom_trans.transform.translation.y = yy_;
     odom_trans.transform.translation.z = 0.0;
     odom_trans.transform.rotation = yaw_quat;
 
     //publish odometry
     nav_msgs::Odometry odometry;
     odometry.header.stamp = ToROSTime(rawOdom.timeStamp);
     odometry.header.frame_id = "world";

     odometry.pose.pose.position.x = xx_;
     odometry.pose.pose.position.y = yy_;
     odometry.pose.pose.position.z = 0.0;
     odometry.pose.pose.orientation = yaw_quat;

     odometry.child_frame_id = "base_link";
     odometry.twist.twist.linear.x = 0.5;
     odometry.twist.twist.linear.y = 0;
     odometry.twist.twist.linear.z = 0;

     odometry.twist.twist.angular.x = 0;
     odometry.twist.twist.angular.y = 0 ;
     odometry.twist.twist.angular.z = 0.8;
     
     odom_pub_.publish(odometry);
     odom_broadcaster_.sendTransform(odom_trans);
     lastReading_.LDist = rawOdom.LDist; lastReading_.RDist = rawOdom.RDist ;lastReading_.Gyro = rawOdom.Gyro;

}
void Node::PublishScan(const ros::WallTimerEvent& event)
{
    if(lds_bridge_ == NULL)
        return;
    LaserFrameData_S scan;
    if(lds_bridge_->GetOneLDSData(&scan, NULL) )
     return;
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = ToROSTime(scan.s32Timestamp);
    scan_msg.header.frame_id = "base_laser";
    scan_msg.angle_min = DEG2RAD(0.0f);
    scan_msg.angle_max = DEG2RAD(360.0f );
    scan_msg.angle_increment =   (scan_msg.angle_max - scan_msg.angle_min) / 1440.0;
    scan_msg.scan_time = 0.125;
    scan_msg.time_increment =scan_msg.scan_time/1440.0;
    scan_msg.range_min = 0.2;
    scan_msg.range_max = 30.0;
    scan_msg.ranges.resize(1440);

    
      for(int i = 0 ; i < 1440 ; ++i)
      {
      	bool InvalidData = false;
      	float dist = scan.Distance[i]/100.0;
          int indexOffset_= -24;
          int jj = 0 ;
      	if(indexOffset_  <=   0)
      	{
                  jj =((-1)*indexOffset_  + i)%1440; 
      	}
      	else
      	{
      	       jj= ((-1)*indexOffset_ + i + 1440)%1440;
      	}
          for(int j = 0 ; j < 3 ; ++j)
      	{
      		if( (jj > _maskDeg_[j][0]) && (jj < _maskDeg_[j][1]) )
      		{
                  dist = 0;
      			break;

      		}
      	}

      	scan_msg.ranges[jj] = dist;

      }
    scan_pub_.publish(scan_msg);


}
void Node::spinForever()
{
	tf2_ros::StaticTransformBroadcaster br_static;
	tf2::Quaternion qua( tf2::Quaternion(1, 0, 0, 0));
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.child_frame_id = "base_laser";
	transformStamped.header.frame_id = "base_link";
	transformStamped.transform.translation.x = -0.08;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.w = qua.getW();
	transformStamped.transform.rotation.x = qua.getX();
	transformStamped.transform.rotation.y = qua.getY();
	transformStamped.transform.rotation.z = qua.getZ();
	transformStamped.header.stamp = ToROSTime(BaseGetTimeTick());
	br_static.sendTransform(transformStamped);

    ros::spin();
}

int  main(int argc, char **argv)
{
    ros::init(argc,argv,"base_move_node");
    Node nn;
   if( nn.initialize()== false)
   {
       ROS_INFO("initial failed");
       return 1;
   }
    nn.spinForever();

    return 0;
}
