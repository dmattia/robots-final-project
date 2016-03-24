#include <python_web_scraper/Vector4.h>
#include <ros/ros.h>

void scoreCallback(const python_web_scraper::Vector4 updatedScores) {
    ROS_INFO("Received an updated score");
    ROS_INFO("Blue score: %d", updatedScores.blue);
    ROS_INFO("Green score: %d", updatedScores.green);
    ROS_INFO("Red score: %d", updatedScores.red);
    ROS_INFO("Yellow score: %d", updatedScores.yellow);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "web_receiver");
   ros::NodeHandle n;
   ros::Subscriber scoreSubscriber = n.subscribe("/web_pub/scores", 10, scoreCallback);
   ros::Rate loop_rate(10);
   
   while(ros::ok())
   {
        ros::spinOnce();
        loop_rate.sleep();
   } 
   return 0;
 }
