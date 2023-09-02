#include <path_creator/path_creator.h>

PathCreator::PathCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("radius", radius_, {5.0});
    private_nh_.param("start_point_x", start_point_x_, {50.0});
    private_nh_.param("start_point_y", start_point_y_, {50.0});
    private_nh_.param("cource_length", cource_length_, {30});
    private_nh_.param("resolution", resolution_, {0.1});

    // Publisher
    pub_target_path_ = nh_.advertise<nav_msgs::Path>("/target_path", 1);

}

// 軌道中の円の中心のx座標を更新
double PathCreator::update_center_x(double center_x)
{
    // 1回目は初期値から円の半径分だけ進める
    // 2回目以降は円の直径分だけ進める
    if(center_x == 0.0)
        return radius_;
    else
        return center_x + 2*radius_;
}

// y座標の正負を判定
bool PathCreator::is_plus_sign(const double x)
{
    // x座標を軌道の半径の4倍の値で割ったときの余りを計算
    // 整数に対する余りは % で求められるが，浮動小数点に関しては fmod を使う
    // fmod(x, y) で x/y の余りを返す．符号はxと同じになる
    double mod = fmod(x, 4*radius_);

    if(mod <= 2*radius_)
        return true;  // プラス
    else
        return false;  // マイナス
}

// 追従軌道のy座標を計算
double PathCreator::calc_cource_y(const double x, const double center_x)
{
    // y座標の絶対値を計算
    // 円の方程式 (x-a)^2 + (y-b)^2 = R^2
    double y = sqrt(pow(radius_, 2) - pow((x-center_x),2));

    // y座標の符号を決定
    if(!(is_plus_sign(x)))
        y *= -1;
    
    return y;
}

// 追従軌道を生成
void PathCreator::create_cource()
{
    //nav_msgs::Path path;
    //target_path_のframe_idをodomにすると，ロボットのスタート地点を(0.0, 0.0)に設定できる？
    target_path_.header.frame_id  = "map";
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    double center_x = 0.0;  // 軌道中の半円の中心

    for(double x=0.0; x<cource_length_; x+=resolution_)
    {
        // 小数点第二位以下を切り捨て，yが虚数になるのを防ぐ
        double round_x =round(x*10);
        x = round_x/10;
        
        pose.pose.position.x = start_point_x_ + x;
        ROS_INFO("fmod = %lf", fmod(x, 2*radius_));  // デバック用
        
        // 半円ができたら，円の中心座標を更新
        // 2個目の条件を足すことで，fmodの値が2*radius_を超えて，円の中心座標の更新がされなくなるのを防ぐ
        if((fmod(x, 2*radius_) <= resolution_/2) || (fmod(x, 2*radius_) >= 2*radius_-resolution_/2))
        {
            center_x = update_center_x(center_x);
            ROS_INFO("center_x = %lf", center_x);  // デバック用
        }

        //if(pow((x-center.x), 2) - pow(radius_, 2) >= 0.0)
            //pose.pose.position.y = 0.0;

        pose.pose.position.y = start_point_y_ + calc_cource_y(x, center_x);
        ROS_INFO("x = %lf, y = %lf", pose.pose.position.x, pose.pose.position.y);  // デバック用


        target_path_.poses.push_back(pose);
    }
}

// メイン文で実行する関数
void PathCreator::process()
{
    ros::Rate loop_rate(hz_);
    create_cource();
    ROS_INFO("Create path finish!");  // デバック用
    
    while(ros::ok())
    {
        target_path_.header.stamp = ros::Time::now();
        pub_target_path_.publish(target_path_);
        loop_rate.sleep();
    }
}

// メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_creator");
    PathCreator pathcreator;
    pathcreator.process();
    return 0;
}