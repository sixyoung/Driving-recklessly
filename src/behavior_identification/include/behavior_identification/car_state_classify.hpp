#include <ros/ros.h>
#include <ros/package.h>
#include <derived_object_msgs/ObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <std_msgs/String.h>
#include <cmath>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <behavior_identification/VehStateSequenceArray.h>
#include <behavior_identification/VehStateSequence.h>

namespace CarStateClassify
{
    class Filter
    {
    public:
        Filter(double dt = 0.05);

        ~Filter() = default;

        // 初始化卡尔曼滤波器状态，输入初始速度和加速度
        void kalmanFilterInit(double vx, double vy, double ax = 0.0, double ay = 0.0);

        // 卡尔曼滤波器预测步骤，输入时间间隔dt
        void KalmanFilterPredict(double dt = 0.05);

        // 卡尔曼滤波器更新步骤，输入测量的速度
        void KalmanFilterUpdate(double measured_vx, double measured_vy);

        // 获取当前估计的加速度大小
        double getAccel();
    private:
    Eigen::Vector4d state_;         ///< [vx, vy, ax, ay]
    Eigen::Matrix4d P_;             ///< 状态协方差矩阵
    Eigen::Matrix4d F_;             ///< 状态转移矩阵
    Eigen::Matrix4d Q_;             ///< 过程噪声协方差矩阵
    Eigen::Matrix<double, 2, 4> H_; ///< 观测矩阵
    Eigen::Matrix2d R_;             ///< 观测噪声协方差矩阵
    Eigen::Matrix4d I_;             ///< 单位矩阵

    double dt_;                      ///< 时间间隔
    };


    class CarStateClassifyNode
    {
    
    public:
        CarStateClassifyNode(ros::NodeHandle& nh);
        ~CarStateClassifyNode() = default;
        void objectArrayCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg);


    private:
        struct VehStateSequence
        {
            uint32_t id = 0;
            std::vector<geometry_msgs::Pose> pose;
            std::vector<geometry_msgs::Twist> twist;
            std::vector<geometry_msgs::Accel> accel;
            std::vector<double> speed ;
            Filter accel_filter;  
        };
        void publishVehStateSequences();

        //用于接受场景状态的回调函数
        void scenarioStatusCallback(const std_msgs::String::ConstPtr& msg);

        ros::Subscriber object_array_sub_;
        ros::Subscriber scenario_status_sub_;
        ros::Publisher veh_state_seq_pub_;
        mutable std::mutex classify_mutex_;
        std::vector<VehStateSequence> veh_state_sequences_;         ///< 存储每辆车的状态序列
        std::vector<uint64_t> time_stamp_sequence_;                 ///< 存储每条消息的时间戳序列      
        derived_object_msgs::ObjectArray latest_object_array_;      ///< 用于接收节点发送msg
        bool is_first_msg_received_ = true;                         ///< 标志是否已经接收到第一条消息
    };
}