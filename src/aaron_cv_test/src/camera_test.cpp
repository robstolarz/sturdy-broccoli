
// Include standard ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraTest
{
	public:
		CameraTest(ros::NodeHandle nh_);
		~CameraTest();

		ros::Subscriber sub;
		ros::Publisher navGoalPublisher;
		tf::TransformListener tfListener;
		tf::StampedTransform transform;
		void show_monitor();
		void processImage();
		cv::Mat *m;
		cv::Mat *edges;
		cv::Point goal;
		bool stillSearching;
	private:
		void imageCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		bool fml;
		std::vector<signed char> *rosmsgdata;
		double mapRes;
		int xOffset, yOffset;
};

int cannyLow = 100;
int cannyHigh = 200;

void f( int, void* ) {}

// Construct our node along with the image transport
CameraTest::CameraTest(ros::NodeHandle nh_)
{
	// Construct here!

	sub = nh_.subscribe("/vo_map", 1, &CameraTest::imageCB, this, ros::TransportHints().reliable());
	navGoalPublisher = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

	stillSearching = true;

	// Create camera preview window
	printf("OpenCV Loaded!\n");
	cv::namedWindow("edgy af", CV_WINDOW_FREERATIO);

	fml = false;
	m = new cv::Mat();
	edges = new cv::Mat();
}

CameraTest::~CameraTest()
{
	cv::destroyWindow("edgy af");
}

void CameraTest::imageCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	try
	{
		std::vector<signed char> data = msg->data;
		rosmsgdata = new std::vector<signed char>(data);

		// signed char *whatever = &rosmsgdata[0];

		unsigned int w = msg->info.width;
		unsigned int h = msg->info.height;

		xOffset = msg->info.origin.position.x;
		yOffset = msg->info.origin.position.y;
		mapRes = msg->info.resolution;

		delete m;
		m = new cv::Mat(w, h, CV_8UC1, rosmsgdata->data());

		fml = true;
		processImage();

	} catch(cv_bridge::Exception& e) {
		// Caught an error
		ROS_ERROR("cv_bridge failed to get image %s", e.what());
		return;
	}
}

void CameraTest::processImage()
{
	if(!fml)
		return;
	if(!m)
		return;
	cv::Mat m1 = m->clone();
	cv::Mat m2 = m->clone();
	cv::bitwise_and(m1, cv::Scalar(1 << 7), m1);
	cv::threshold(m2, m2, 0, 255, cv::THRESH_BINARY);
	cv::Canny(m1, m1, (double)100, (double)cannyHigh, 3);
	cv::Canny(m2, m2, (double)100, (double)cannyHigh, 3);
	cv::bitwise_and(m1, m2, *edges);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(*edges, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);


	cv::Point nearestPoint;
	double nearestDistance = -1;

 	int x = (transform.getOrigin().x()-xOffset)/mapRes;
	int y = (transform.getOrigin().y()-yOffset)/mapRes;
	cv::circle(*edges, cv::Point(x,y), 20, cv::Scalar(255,0,0));
	
	stillSearching = false;
	for (unsigned int i = 0; i < contours.size(); i++) {
		// shitty comment
		for (unsigned int j = 0; j < contours[i].size() && contours[i].size() > 50; j++ ) {
			double distance = sqrt(pow(x-contours[i][j].x,2) + pow(y-contours[i][j].y,2));
			if((distance < nearestDistance && distance > 1) || nearestDistance < 0) {
				nearestDistance = distance;
				nearestPoint = contours[i][j];
				stillSearching = true;
			}
			//cv::circle(*edges, contours[i][j], 4, cv::Scalar(255,0,0));
		}
	}
	if(stillSearching) {
		if(nearestPoint.x != goal.x && nearestPoint.y != goal.y) {
			goal.x = nearestPoint.x;
			goal.y = nearestPoint.y;

			geometry_msgs::PoseStamped msg;

			msg.pose.position.x = goal.x*mapRes+xOffset;
			msg.pose.position.y = goal.y*mapRes+xOffset;
			msg.pose.orientation.w = 1;
			msg.header.frame_id = "map";
			navGoalPublisher.publish(msg);

		}
	} else {

		goal.x = 999;
		goal.y = 999;

		geometry_msgs::PoseStamped msg;

		msg.pose.position.x = goal.x*mapRes+xOffset;
		msg.pose.position.y = goal.y*mapRes+xOffset;
		msg.pose.orientation.w = 1;
		msg.header.frame_id = "map";
		navGoalPublisher.publish(msg);

		printf("done?\n");
	}

	printf("goal, x: %i, y: %i \n", goal.x, goal.y);
	printf("current, x: %i, y: %i \n", x, y);
	cv::circle(*edges, goal, 10, cv::Scalar(255,0,0));
	
}

void CameraTest::show_monitor()
{
	if(!fml)
		return;
	if(!m)
		return;
	cv::imshow("edgy af", *edges);
	cv::waitKey(1);
}

// ROS main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "CameraTest");
	ros::NodeHandle nh;
	CameraTest cam(nh);
	while(cam.stillSearching) {
		try{
			cam.tfListener.lookupTransform("/map", "/base_footprint", ros::Time(0), cam.transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		cam.processImage();
		cam.show_monitor(); 	
	}
}
