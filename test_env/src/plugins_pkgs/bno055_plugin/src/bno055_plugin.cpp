#include "bno055_plugin.hpp"
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>  // For tf::createQuaternionMsgFromRollPitchYaw
#include <boost/make_shared.hpp>

namespace gazebo
{
    namespace bno055
    {
        BNO055::BNO055() : acc_bias_x(0.0), acc_bias_y(0.0), acc_bias_z(0.0),
                           gyro_bias_x(0.0), gyro_bias_y(0.0), gyro_bias_z(0.0),
                           previous_acc_x(0.0), previous_acc_y(0.0), previous_acc_z(0.0),
                           alpha(0.95) {}

        void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            this->m_model = model_ptr;

            // ROS 초기화
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client_bno", ros::init_options::NoSigintHandler);
            }

            // ROS 노드 핸들 초기화
            this->nh = boost::make_shared<ros::NodeHandle>();
            this->m_pubBNO = nh->advertise<sensor_msgs::Imu>("/automobile/IMU", 10);

            // Gazebo의 WorldUpdateBegin 이벤트에 연결
            this->update_connection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BNO055::OnUpdate, this));
        }

        void BNO055::OnUpdate()
        {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "automobile_chassis_link";

        // 1) Get world-frame acceleration
        ignition::math::Vector3d acc_world = this->m_model->WorldLinearAccel();

        // 2) Transform world -> body
        ignition::math::Quaterniond bodyRot = this->m_model->RelativePose().Rot().Inverse();
        ignition::math::Vector3d acc_body  = bodyRot.RotateVector(acc_world);

        // 3) Assign to message
        imu_msg.linear_acceleration.x = acc_body.X() - acc_bias_x;
        imu_msg.linear_acceleration.y = acc_body.Y() - acc_bias_y;
        imu_msg.linear_acceleration.z = acc_body.Z() - acc_bias_z;

        // Angular velocity (relative)
        ignition::math::Vector3d gyro_rel = this->m_model->RelativeAngularVel();
        imu_msg.angular_velocity.x = gyro_rel.X() - gyro_bias_x;
        imu_msg.angular_velocity.y = gyro_rel.Y() - gyro_bias_y;
        imu_msg.angular_velocity.z = gyro_rel.Z() - gyro_bias_z;

        // Orientation (relative)
        double roll = this->m_model->RelativePose().Rot().Roll();
        double pitch= this->m_model->RelativePose().Rot().Pitch();
        double yaw  = this->m_model->RelativePose().Rot().Yaw();
        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(q, imu_msg.orientation);

        // Covariances
        imu_msg.orientation_covariance[0] = 0.005;
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[0] = 0.005;

        this->m_pubBNO.publish(imu_msg);
        }

    }  // namespace bno055
}  // namespace gazebo

GZ_REGISTER_MODEL_PLUGIN(gazebo::bno055::BNO055)
