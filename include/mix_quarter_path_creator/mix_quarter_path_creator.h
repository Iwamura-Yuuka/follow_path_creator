#ifndef MIX_QUARTER_PATH_CREATOR_H
#define MIX_QUARTER_PATH_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class MixQuarterPathCreator
{
public:
    MixQuarterPathCreator();
    void process();

private:
    // ----- 関数 (引数あり) -----
    double update_center_x(double center_x);                      // 軌道中の円の中心のx座標を更新
    bool is_plus_sign(const double x);                            // y座標の正負を判定
    double calc_cource_y(const double x, const double center_x, const double center_y);  // 追従軌道のy座標を計算

    // ----- 関数 (引数なし) -----
    void create_cource();                                         // 追従軌道を生成

    // ----- 変数 -----
    // yamlファイルで設定可能な変数
    int hz_;                          // ループ周波数 [Hz]
    int semicircle_number_;           // 軌道に含まれる半円の個数
    int semicircle_counter_;          // 半円の個数をカウント
    std::vector<double> all_radius_;  // 目標軌道に含まれるすべての旋回半径 [m]
    double radius_;                   // 各旋回半径代入用 [m]
    double tmp_radius_;              // center_x計算用 [m]
    double init_x_;         // スタート地点のx座標
    double init_y_;         // スタート地点のy座標
    double init_theta_;     // 生成する軌道の向き
    double max_cource_length_;       // 目標軌道の長さの最大値 [m]
    double cource_length_;           // x軸方向の目標軌道の長さの計算用 [m]
    double resolution_;              // 軌道生成時の刻み幅 [m]


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publisher
    ros::Publisher pub_target_path_;

    // 各種オブジェクト
    nav_msgs::Path target_path_;  // 目標軌道
};


#endif // MIX_QUARTER_PATH_CREATOR_H