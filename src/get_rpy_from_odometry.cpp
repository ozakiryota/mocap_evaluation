#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

class GetRPYFromOdometry{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscriber*/
		ros::Subscriber sub_odom;
		/*publisher*/
		ros::Publisher pub_rpy;
		/*objects*/
	public:
		GetRPYFromOdometry();
		void CallbackPose(const nav_msgs::OdometryConstPtr& msg);
};

GetRPYFromOdometry::GetRPYFromOdometry()
{
	sub_odom = nh.subscribe("/gyrodometry", 1, &GetRPYFromOdometry::CallbackPose, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_est", 1);
}

void GetRPYFromOdometry::CallbackPose(const nav_msgs::OdometryConstPtr& msg)
{
	tf::Quaternion q_orientation;
	quaternionMsgToTF(msg->pose.pose.orientation, q_orientation);
	std_msgs::Float64MultiArray rpy_pub;	//[deg]
	rpy_pub.data.resize(3);
	tf::Matrix3x3(q_orientation).getRPY(rpy_pub.data[0], rpy_pub.data[1], rpy_pub.data[2]);
	for(int i=0;i<3;i++)	rpy_pub.data[i] = rpy_pub.data[i]/M_PI*180.0;
	pub_rpy.publish(rpy_pub);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_rpy_from_odometry");

	GetRPYFromOdometry get_rpy_from_odometry;

	ros::spin();
}
