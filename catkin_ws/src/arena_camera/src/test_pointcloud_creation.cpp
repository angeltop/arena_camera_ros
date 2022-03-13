
#include <ros/ros.h>
#include <arena_camera/arena_camera_node.h>

ros::Publisher pc_pub_;

void subcribe(const sensor_msgs::Image::ConstPtr &msg)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cv::Mat im;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{		
		if(msg->encoding ==sensor_msgs::image_encodings::COORD3D_ABC16 ||
			msg->encoding==sensor_msgs::image_encodings::TYPE_16UC3)
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC3);
			im = cv_ptr->image;
			cv::Mat channels[3];
			cv::split(im, channels);
			cv::Mat x, y, z;
			channels[0].convertTo(x, CV_64F, 0.001,0);
			channels[1].convertTo(y, CV_64F, 0.001,0);
			channels[2].convertTo(z, CV_64F, 0.001,0);
			
			for(int i=0; i<x.rows; i++)
				for(int j=0; j<x.cols; j++)
				{
					pcl::PointXYZ point(x.at<double>(i,j), y.at<double>(i,j), z.at<double>(i,j));
					cloud->points.push_back(point);	
				}
			cloud->height = msg->height;
			cloud->width = msg->width;
			cloud->header.frame_id = msg->header.frame_id;
			pc_pub_.publish(cloud);
		}
		else if(msg->encoding ==sensor_msgs::image_encodings::COORD3D_ABCY16 ||
			msg->encoding==sensor_msgs::image_encodings::TYPE_16UC4
		)
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC4);
			im = cv_ptr->image;
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
			cv::Mat channels[4];
			cv::split(im, channels);
			cv::Mat x, y, z, I;
			channels[0].convertTo(x, CV_32F, 0.001,0);
			channels[1].convertTo(y, CV_32F, 0.001,0);
			channels[2].convertTo(z, CV_32F, 0.001,0);
			channels[3].convertTo(I, CV_32F, 0, 0);
			for(int i=0; i<x.rows; i++)
				for(int j=0; j<x.cols; j++)
				{
					pcl::PointXYZI point(I.at<float>(i,j));
					point.x = x.at<float>(i,j);
					point.y = y.at<float>(i,j);
					point.z = z.at<float>(i,j);
					cloud->points.push_back(point);	
				}
			cloud->height = msg->height;
			cloud->width = msg->width;
			cloud->header.frame_id = msg->header.frame_id;
			pc_pub_.publish(cloud);

		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		
	}	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_pointcloud");
	
	ros::NodeHandle nh("~");
	ros::Subscriber sub = nh.subscribe("/arena_camera_node/image_raw", 0, subcribe);
	
	pc_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/arena_camera_node/points", 0);
	
	ros::spin();
	
	return 0;
	
}
