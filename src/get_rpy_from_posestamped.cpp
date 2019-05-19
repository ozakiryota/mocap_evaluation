#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

class GetRPYFromPoseStamped{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscriber*/
		ros::Subscriber sub_pose;
		/*publisher*/
		ros::Publisher pub_rpy;
		/*objects*/
	public:
		GetRPYFromPoseStamped();
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
};

GetRPYFromPoseStamped::GetRPYFromPoseStamped()
{
	sub_pose = nh.subscribe("/pose_mocap", 1, &GetRPYFromPoseStamped::CallbackPose, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_mocap", 1);
}

void GetRPYFromPoseStamped::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	tf::Quaternion q_orientation;
	quaternionMsgToTF(msg->pose.orientation, q_orientation);
	std_msgs::Float64MultiArray rpy_pub;	//[deg]
	rpy_pub.data.resize(3);
	tf::Matrix3x3(q_orientation).getRPY(rpy_pub.data[0], rpy_pub.data[1], rpy_pub.data[2]);
	for(int i=0;i<3;i++)	rpy_pub.data[i] = rpy_pub.data[i]/M_PI*180.0;
	pub_rpy.publish(rpy_pub);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_rpy_from_posestamped");

	GetRPYFromPoseStamped get_rpy_from_posestamped;

	ros::spin();
}
