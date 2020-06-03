#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
using namespace std;

ros::Publisher filtered_scan_pub;
ros::Subscriber input_scan_sub;
sensor_msgs::LaserScan filtered_scan;

bool my_update(const sensor_msgs::LaserScan& input_scan)
{
  
  filtered_scan.ranges.resize(input_scan.ranges.size());
  filtered_scan.intensities.resize(input_scan.intensities.size());

  unsigned int n = input_scan.ranges.size();

  //loop through the scan and reassign range values 
  for (unsigned int i = 0; i < n; ++i){
    if(input_scan.ranges[i] > 5.0 ||
       isnan(input_scan.ranges[i]) ||
       isinf(input_scan.ranges[i])) {
      filtered_scan.ranges[i] = 5.0;
      filtered_scan.intensities[i] = input_scan.intensities[i];
    }
    else {
      filtered_scan.ranges[i] = input_scan.ranges[i];
      filtered_scan.intensities[i] = input_scan.intensities[i];
    }
  }
 
  //make sure to set all the needed fields on the filtered scan
  filtered_scan.header.frame_id = input_scan.header.frame_id;
  filtered_scan.header.stamp = input_scan.header.stamp;
  filtered_scan.angle_min = input_scan.angle_min;
  filtered_scan.angle_max = input_scan.angle_max;
  filtered_scan.angle_increment = input_scan.angle_increment;
  filtered_scan.time_increment = input_scan.time_increment;
  filtered_scan.scan_time = input_scan.scan_time;
  filtered_scan.range_min = input_scan.range_min;
  filtered_scan.range_max = input_scan.range_max;
 
  return true;
}

void input_scan_sub_callback(const sensor_msgs::LaserScan& input_scan)
{
  my_update(input_scan);
  filtered_scan_pub.publish(filtered_scan);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "scan_filter");
  ros::NodeHandle n1, n2;
  input_scan_sub = n1.subscribe("/scan", 10, input_scan_sub_callback);
  filtered_scan_pub = n2.advertise<sensor_msgs::LaserScan>("filtered_scan", 50);
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
