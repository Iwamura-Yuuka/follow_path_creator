#include <fusion_path_creator/fusion_path_creator.h>

FusionPathCreator::FusionPathCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("semicircle_number", semicircle_number_, {10});
    private_nh_.param("semicircle_counter", semicircle_counter_, {0});
    private_nh_.getParam("all_radius", all_radius_);  // ここだけはyamlファイルで必ず指定する
    private_nh_.param("radius", radius_, {0.0});
    private_nh_.param("tmp_radius", tmp_radius_, {0.0});
    private_nh_.param("start_point_x", start_point_x_, {0.0});
    private_nh_.param("start_point_y", start_point_y_, {0.0});
    private_nh_.param("max_cource_length", max_cource_length_, {50});
    private_nh_.param("cource_length", cource_length_, {0.0});
    private_nh_.param("resolution", resolution_, {0.1});

    // Publisher
    pub_target_path_ = nh_.advertise<nav_msgs::Path>("/target_path", 1);

}

// 軌道中の円の中心のx座標を更新
double FusionPathCreator::update_center_x(double center_x)
{
    // 1回目は初期値から円の半径分だけ進める
    // 2回目以降は1つ前の旋回半径と今の旋回半径を足した分だけ進める
    if(center_x == 0.0)
        return radius_;
    else
        return center_x + tmp_radius_ + radius_;
}

// y座標の正負を判定
bool FusionPathCreator::is_plus_sign(const double x)
{
    // x座標を軌道の半径の4倍の値で割ったときの余りを計算
    // 整数に対する余りは % で求められるが，浮動小数点に関しては fmod を使う
    // fmod(x, y) で x/y の余りを返す．符号はxと同じになる
    double mod = fmod(x, 4*radius_);

    if(semicircle_counter_ % 2 == 0)
        return true;   // プラス
    else
        return false;  // マイナス
}

// 追従軌道のy座標を計算
double FusionPathCreator::calc_cource_y(const double x, const double center_x)
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
void FusionPathCreator::create_cource()
{
    //nav_msgs::Path path;
    //target_path_のframe_idをodomにすると，ロボットのスタート地点を(0.0, 0.0)に設定できる？
    target_path_.header.frame_id  = "map";
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    // radius_に0.0が入っているとfmodの計算でエラーが出てしまうので，最初の旋回半径を代入しておく
    radius_ = all_radius_[0];

    // 軌道中の半円の中心を最初の旋回半径に合わせて初期化
    double center_x = all_radius_[0];

    double tmp_radius = 0.0;  // center_x計算用
    double update_check = 0.0;  // 旋回半径，円の中心座標の更新判定用

    double x;

    for(x=0.0; x<max_cource_length_; x+=resolution_)
    {
        // 小数点第二位以下を切り捨て，yが虚数になるのを防ぐ
        double round_x =round(x*10);
        x = round_x/10;
        
        pose.pose.position.x = start_point_x_ + x;
        // ROS_INFO("fmod = %lf", fmod(x, 2*radius_));  // デバック用
        
        // 半円ができたら，旋回半径，円の中心座標を更新
        if((update_check >= 2*radius_-resolution_/2) && (update_check <= 2*radius_+resolution_/2))
        {
            // 半円の個数のカウントを増やす
            semicircle_counter_ ++;

            // 1つ前の旋回半径を格納
            tmp_radius_ = radius_;

            // 旋回半径を更新
            radius_ = all_radius_[semicircle_counter_];
            
            // 円の中心座標を更新
            center_x = update_center_x(center_x);
            ROS_INFO("center_x = %lf", center_x);  // デバック用
            ROS_INFO("Now x is %lf", x);           // デバック用

            // x軸方向の目標軌道の長さを計算
            cource_length_ += 2*radius_;

            // 旋回半径，円の中心座標を更新判定のカウントを0に戻す
            update_check = 0.0;
        }

        // 終了判定
        if(semicircle_counter_ > semicircle_number_)
            break;

        pose.pose.position.y = start_point_y_ + calc_cource_y(x, center_x);
        ROS_INFO("x = %lf, y = %lf", pose.pose.position.x, pose.pose.position.y);  // デバック用

        // 計算した座標を格納
        target_path_.poses.push_back(pose);

        // 旋回半径，円の中心座標を更新判定を軌道の刻み幅分だけ進める
        update_check += resolution_;
    }
    
    // 指定された旋回直径の合計が目標軌道の長さの最大値を超えた場合
    if(x >= max_cource_length_)
        cource_length_ = max_cource_length_;

}

// メイン文で実行する関数
void FusionPathCreator::process()
{
    ros::Rate loop_rate(hz_);
    create_cource();
    // ROS_INFO("Create path finish!");  // デバック用

    // 生成した軌道の長さを表示
    ROS_INFO("Path length is %lf [m]", cource_length_ );
    
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
    ros::init(argc, argv, "fusion_path_creator");
    FusionPathCreator f_pathcreator;
    f_pathcreator.process();
    return 0;
}