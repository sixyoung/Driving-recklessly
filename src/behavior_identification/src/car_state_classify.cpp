#include "behavior_identification/car_state_classify.hpp"

namespace CarStateClassify
{
    CarStateClassifyNode::CarStateClassifyNode(ros::NodeHandle& nh)
    {
        object_array_sub_ = nh.subscribe(
            "/carla/objects",
            10,
            &CarStateClassifyNode::objectArrayCallback,
            this
        );

        veh_state_seq_pub_ = nh.advertise<behavior_identification::VehStateSequenceArray>(
            "/veh_state_sequences",
            10
        );

        scenario_status_sub_ = nh.subscribe(
            "/scenario_status", 
            10, 
            &CarStateClassifyNode::scenarioStatusCallback, 
            this
        );
    }

    void CarStateClassifyNode::objectArrayCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg)
    {
        {
            std::lock_guard<std::mutex> lock(classify_mutex_);
            latest_object_array_ = *msg;
            
        }
        uint64_t current_time= msg->header.stamp.toNSec();

        auto low_pass_filter = [](double current_accel, double last_accel, double alpha = 0.05) {
            return alpha * current_accel + (1 - alpha) * last_accel;
        };

        // 统一在开头计算与上一帧的时间差（秒）
        double delta_t_sec = 0.0;
        if (!time_stamp_sequence_.empty())
        {
            // delta_t_sec = (current_time - time_stamp_sequence_.back()) * 1e-9;
            delta_t_sec = (current_time - time_stamp_sequence_.back()) * 1e-9;
        }

        delta_t_sec = 0.05;

        time_stamp_sequence_.push_back(current_time);

        if(time_stamp_sequence_.size() > 10)
        {
            time_stamp_sequence_.erase(time_stamp_sequence_.begin(), 
             time_stamp_sequence_.begin() + (time_stamp_sequence_.size() - 10));
        }

        derived_object_msgs::ObjectArray object = latest_object_array_;
        for (auto &obj: object.objects)
        {
            double speed = std::sqrt(
			    obj.twist.linear.x * obj.twist.linear.x +
			    obj.twist.linear.y * obj.twist.linear.y +
			    obj.twist.linear.z * obj.twist.linear.z);

            double accel_val = 0.0; 
            
            // 查找该车是否已经在历史序列中
            auto it = std::find_if(veh_state_sequences_.begin(), veh_state_sequences_.end(), 
                                   [&obj](const auto& seq){ return seq.id == obj.id; });

            if (it == veh_state_sequences_.end())
            {
                // 如果是新车，新建一个对象记录
                VehStateSequence obj_sequence;
                obj_sequence.id = obj.id;
                obj_sequence.pose.push_back(obj.pose);
                obj_sequence.twist.push_back(obj.twist);

                obj_sequence.accel_filter = Filter(delta_t_sec);
                obj_sequence.accel_filter.kalmanFilterInit(obj.twist.linear.x, obj.twist.linear.y);
                
                geometry_msgs::Accel accel_msg;
                // 用 x 方向临时存储标量加速度
                accel_msg.linear.x = accel_val; 
                obj_sequence.accel.push_back(accel_msg);
                
                obj_sequence.speed.push_back(speed);
                veh_state_sequences_.push_back(obj_sequence);
            }
            else
            {
                // 该车已存在，更新数据
                if (delta_t_sec > 1e-6)
                {
                    speed = low_pass_filter(speed, it->speed.back(), 0.05);
                    accel_val = (speed - it->speed.back()) / delta_t_sec;
                    // 对加速度值进行低通滤波
                    accel_val = low_pass_filter(accel_val, it->accel.back().linear.x); 
                    // 更新卡尔曼滤波器
                    // it->accel_filter.KalmanFilterPredict(delta_t_sec);
                    // it->accel_filter.KalmanFilterUpdate(obj.twist.linear.x, obj.twist.linear.y);
                    // accel_val = it->accel_filter.getAccel();
                }

                geometry_msgs::Accel accel_msg;
                accel_msg.linear.x = accel_val;

                it->pose.push_back(obj.pose);
                it->twist.push_back(obj.twist);
                it->accel.push_back(accel_msg);
                it->speed.push_back(speed);
            }
        }
        
        is_first_msg_received_ = false;
        publishVehStateSequences();
    }

    void CarStateClassifyNode::publishVehStateSequences()
    {
        behavior_identification::VehStateSequenceArray out_msg;

        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "map";

        {
            std::lock_guard<std::mutex> lock(classify_mutex_);

            out_msg.vehicles.reserve(veh_state_sequences_.size());

            for (const auto& veh_seq : veh_state_sequences_)
            {
                behavior_identification::VehStateSequence veh_msg;

                veh_msg.id = veh_seq.id;
                veh_msg.pose = veh_seq.pose;
                veh_msg.twist = veh_seq.twist;
                veh_msg.accel = veh_seq.accel;
                veh_msg.speed = veh_seq.speed;

                out_msg.vehicles.push_back(veh_msg);
            }
        }

        veh_state_seq_pub_.publish(out_msg);
    }

    void CarStateClassifyNode::scenarioStatusCallback(const std_msgs::String::ConstPtr& msg)
    {
        // 收到消息后直接清空轨迹，无差别的不再做 "start" 等关键词的判断
        std::lock_guard<std::mutex> lock(classify_mutex_);
        // 发生变化，则清空所有缓存的车辆状态及历史轨迹
        veh_state_sequences_.clear();
        is_first_msg_received_ = true;
        
        ROS_INFO("===== Scenario Change Detected! Cleared vehicle state sequences. =====");
    }


    Filter::Filter(double dt)
    {
        this->dt_ = dt;
        state_.setZero();

        F_.setIdentity();
        F_(0, 2) = dt_;
        F_(1, 3) = dt_;

        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;

        Q_.setIdentity();
        Q_(0, 0) = 0.01;
        Q_(1, 1) = 0.01;
        Q_(2, 2) = 0.1;
        Q_(3, 3) = 0.1;

        R_.setIdentity();
        R_(0, 0) = 0.1;
        R_(1, 1) = 0.1;
        
        P_.setIdentity();
        P_ *= 1.0;

        I_.setIdentity();
    }

    void Filter::kalmanFilterInit(double vx, double vy, double ax, double ay)
    {
        state_ << vx, vy, ax, ay;
    }

    void Filter::KalmanFilterPredict(double dt)
    {
        dt_ = dt;
        state_ = F_ * state_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void Filter::KalmanFilterUpdate(double measured_vx, double measured_vy)
    {
        Eigen::Vector2d z(measured_vx, measured_vy);
        Eigen::Vector2d y = z - H_ * state_;
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

        state_ = state_ + K * y;
        P_ = (I_ - K * H_) * P_;
    }

    double Filter::getAccel()
    {
        double ax = state_(2); 
        double ay = state_(3); 
        return std::sqrt(ax * ax + ay * ay);
    }
}



