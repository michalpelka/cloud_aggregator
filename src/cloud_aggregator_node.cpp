#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/voxel_grid.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



class aggregator_node
{
public:
	aggregator_node():
		nh("~"),tfListener(tfBuffer)
	{
		aggregateNumber = 20;
		targetFrame="base_link";

		nh.getParam("targetFrame", targetFrame);
		nh.getParam("aggregateNumber", aggregateNumber);
		cloud_sub= nh.subscribe<PointCloud>("/input", 5, &aggregator_node::callback, this);
		cloud_pub= nh.advertise<PointCloud>("output", 1);
		ros::spin();
	}


private:
	void callback(const PointCloud::ConstPtr& msg);
	ros::Subscriber cloud_sub;
	ros::Publisher cloud_pub;
	ros::NodeHandle nh;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;

	std::deque<PointCloud::Ptr> clouds;
	int aggregateNumber;
	std::string targetFrame;

};


void aggregator_node::callback(const PointCloud::ConstPtr& msg)
{
	try {

		ros::Time transformationTime;
		pcl_conversions::fromPCL(msg->header.stamp,transformationTime);

		Eigen::Affine3d transformStampedAffineD = Eigen::Affine3d::Identity();




		geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
				"odom",
				msg->header.frame_id,
				transformationTime, ros::Duration(0.5));
		tf::transformMsgToEigen(transformStamped.transform, transformStampedAffineD);


		PointCloud::Ptr cloudFiltered(new PointCloud);
		PointCloud::Ptr cloudTransformed (new PointCloud);

		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (msg);
		sor.setLeafSize (0.1f, 0.1f, 0.1f);
		sor.filter (*cloudFiltered);

		pcl::transformPointCloud(*cloudFiltered, *cloudTransformed, transformStampedAffineD.cast<float>());

		clouds.push_back(cloudTransformed);



		/// publishing

		geometry_msgs::TransformStamped transformOdomBaseLink = tfBuffer.lookupTransform(
						targetFrame,
						"odom",
						ros::Time(0), ros::Duration(0.5));

		Eigen::Affine3d transformOdomBaseLinkEigen;
		tf::transformMsgToEigen(transformOdomBaseLink.transform, transformOdomBaseLinkEigen);

		PointCloud cloud_to_pub;

		for (int i =0; i<clouds.size(); i++ )
		{
			ros::Time getTime;
			cloud_to_pub +=*clouds[i];
			pcl_conversions::fromPCL(clouds[i]->header.stamp,getTime);
			double ageSec = (ros::Time::now()-getTime).toSec();
			ROS_INFO_STREAM("ageSec : "<< ageSec);
		}
		if (clouds.size()>aggregateNumber)
		{
			clouds.pop_front();
		}
		cloud_to_pub.header.frame_id=targetFrame;
		pcl::transformPointCloud(cloud_to_pub, cloud_to_pub, transformOdomBaseLinkEigen.cast<float>());
		cloud_pub.publish(cloud_to_pub);

	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN_STREAM(ex.what());

	}




}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  aggregator_node();

}
