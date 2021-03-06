#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class ResetMocapOrigin{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle _nhPrivate;
		/*listener*/
		tf::TransformListener listener;
		/*publisher*/
		ros::Publisher pub_pose;
		/*objects*/
		tf::Quaternion q_ini_position;
		tf::Quaternion q_ini_orientation;
		tf::Quaternion q_raw_position;
		tf::Quaternion q_raw_orientation;
		tf::Quaternion q_relative_position;
		tf::Quaternion q_relative_orientation;
		/*time*/
		ros::Time time_pub;
		/*flags*/
		bool inipose_is_available = false;
		/*parameter*/
		std::string _frame_id_gt;
	public:
		ResetMocapOrigin();
		void GetMocapTF(void);
		void Publication(void);
};

ResetMocapOrigin::ResetMocapOrigin()
	: _nhPrivate("~")
{
	std::cout << "--- reset_mocap_origin ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("frame_id_gt", _frame_id_gt, std::string("/vicon/sensors/sensors"));
	std::cout << "_frame_id_gt = " << _frame_id_gt << std::endl;
	/*publisher*/
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_mocap", 1);
}

void ResetMocapOrigin::GetMocapTF(void)
{
	try{
		tf::StampedTransform transform;
		// listener.lookupTransform("/vicon/infant/infant", "/world", ros::Time(0), transform);
		listener.lookupTransform("/world", _frame_id_gt, ros::Time(0), transform);
		time_pub = ros::Time(0);
		q_raw_position = tf::Quaternion(
			transform.getOrigin().x(),
			transform.getOrigin().y(),
			transform.getOrigin().z(),
			0.0);
		q_raw_orientation = transform.getRotation();
		if(!inipose_is_available){
			q_ini_position = q_raw_position;
			q_ini_orientation = q_raw_orientation;
			inipose_is_available = true;
		}
		else{
			q_relative_position = tf::Quaternion(
				q_raw_position.x() - q_ini_position.x(),
				q_raw_position.y() - q_ini_position.y(),
				q_raw_position.z() - q_ini_position.z(),
				0.0
			);
			q_relative_position = q_ini_orientation.inverse()*q_relative_position*q_ini_orientation;
			q_relative_orientation = q_ini_orientation.inverse()*q_raw_orientation;
		}
		Publication();
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

void ResetMocapOrigin::Publication(void)
{
	geometry_msgs::PoseStamped pose_pub;
	pose_pub.header.frame_id = "/world";
	pose_pub.header.stamp = time_pub;
	pose_pub.pose.position.x = q_relative_position.x();
	pose_pub.pose.position.y = q_relative_position.y();
	pose_pub.pose.position.z = q_relative_position.z();
	quaternionTFToMsg(q_relative_orientation, pose_pub.pose.orientation);
	pub_pose.publish(pose_pub);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "reset_mocap_origin");

	ResetMocapOrigin reset_mocap_origin;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		reset_mocap_origin.GetMocapTF();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
