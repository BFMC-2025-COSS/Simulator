#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "utils/IMU.h"

namespace gazebo
{
    namespace bno055
    {
        class BNO055 : public ModelPlugin
        {
        public:
            BNO055();
            virtual void Load(physics::ModelPtr, sdf::ElementPtr);
            void OnUpdate();

        private:
            // Gazebo 관련 멤버 변수
            physics::ModelPtr m_model;

            // ROS 관련 멤버 변수
            boost::shared_ptr<ros::NodeHandle> nh;
            ros::Publisher m_pubBNO;

            // Gazebo의 업데이트 이벤트 연결
            event::ConnectionPtr update_connection;

            // IMU 관련 데이터 처리
            utils::IMU m_bno055_pose;

            // 바이어스 및 필터 변수
            double acc_bias_x, acc_bias_y, acc_bias_z;
            double gyro_bias_x, gyro_bias_y, gyro_bias_z;
            double previous_acc_x, previous_acc_y, previous_acc_z;
            double alpha;
        };
    }  // namespace bno055
}  // namespace gazebo
