#ifndef TEST_PATH_CREATOR_H
#define TEST_PATH_CREATOR_H

#include<ros/ros.h>
#include<nav_msgs/Path.h>

class TestPathCreator
{
public:
    TestPathCreator();
    ~TestPathCreator();
    void process();

private:
    void create_course();
    double course_function(double x);

    int hz_;
    double resolution_;
    double course_length_;
    double init_x_;
    double init_y_;
    double init_theta_;
    double A1_, A2_, A3_;
    double omega1_, omega2_, omega3_;
    double delta1_, delta2_, delta3_;
    nav_msgs::Path test_path_;
    std::string world_frame_id_;

    ros::Publisher pub_path_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
};

#endif //TEST_PATH_CREATOR_H