#ifndef QUARTER_PATH_CREATOR_H
#define QUARTER_PATH_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class QuarterPathCreator
{
public:
    QuarterPathCreator();
    void process();

private:
    // ----- 関数 (引数あり) -----
    double update_center_x(double center_x);                      // 軌道中の円の中心のx座標を更新
    bool is_plus_sign(const double center_y);                            // y座標の正負を判定
    double calc_cource_y(const double x, const double center_x, const double center_y);  // 追従軌道のy座標を計算

    // ----- 関数 (引数なし) -----
    void create_cource();                                         // 追従軌道を生成

    // ----- 変数 -----
    // yamlファイルで設定可能な変数
    int hz_;                // ループ周波数 [Hz]
    double radius_;         // 目標軌道の旋回半径
    double start_point_x_;  // スタート地点のx座標
    double start_point_y_;  // スタート地点のy座標
    double cource_length_;  // 目標軌道の長さ
    double resolution_;     // 軌道生成時の刻み幅

    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publisher
    ros::Publisher pub_target_path_;

    // 各種オブジェクト
    nav_msgs::Path target_path_;  // 目標軌道
};


#endif // QUARTER_PATH_CREATOR_H
