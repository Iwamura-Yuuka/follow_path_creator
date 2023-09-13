#include<test_path_creator/test_path_creator.h>

TestPathCreator::TestPathCreator(): nh_(""), private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("resolution", resolution_, {0.1});
    private_nh_.param("course_length", course_length_, {30});
    private_nh_.param("init_x", init_x_, {0.0});
    private_nh_.param("init_y", init_y_, {0.0});
    private_nh_.param("init_theta", init_theta_, {0.0});
    private_nh_.param("A1", A1_, {0.0});
    private_nh_.param("omega1", omega1_, {0.0});
    private_nh_.param("delta1", delta1_, {0.0});
    private_nh_.param("A2", A2_, {0.0});
    private_nh_.param("omega2", omega2_, {0.0});
    private_nh_.param("delta2", delta2_, {0.0});
    private_nh_.param("A3", A3_, {0.0});
    private_nh_.param("omega3", omega3_, {0.0});
    private_nh_.param("delta3", delta3_, {0.0});
    private_nh_.param("world_frame_id", world_frame_id_, {"odom"});

    omega1_ = omega1_*M_PI/180;
    delta1_ = delta1_*M_PI/180;
    omega2_ = omega2_*M_PI/180;
    delta2_ = delta2_*M_PI/180;
    omega3_ = omega3_*M_PI/180;
    delta3_ = delta3_*M_PI/180;

    pub_path_ = nh_.advertise<nav_msgs::Path>("/target_path", 1);
}

TestPathCreator::~TestPathCreator(){;}

void TestPathCreator::create_course()
{
    nav_msgs::Path path;
    path.header.frame_id = world_frame_id_;
    for(double x=0.0; x<course_length_; x+=resolution_)
    {
        geometry_msgs::PoseStamped pose;
        double y = course_function(x);
        double r = sqrt(x*x + y*y);
        double theta = atan2(y,x);
        pose.header.frame_id = world_frame_id_;
        pose.pose.position.x = init_x_ + r*cos(theta + init_theta_);
        pose.pose.position.y = init_y_ + r*sin(theta + init_theta_);
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
        std::cout<<pose.pose.position.x<<","<<pose.pose.position.y<<std::endl;
    }

    test_path_ = path;
}

double TestPathCreator::course_function(double x)
{
    return A1_*sin(omega1_*x + delta1_) + A2_*sin(omega2_*x + delta2_) + A3_*sin(omega3_*x + delta3_);
}

void TestPathCreator::process()
{
    ros::Rate loop_rate(hz_);
    create_course();
    while(ros::ok()){
        test_path_.header.stamp = ros::Time::now();
        pub_path_.publish(test_path_);
        // std::cout<<"hoge"<<std::endl;
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_path_creator");
    TestPathCreator test_path_creator;
    test_path_creator.process();
}