
#include "usv_controller/USVTransformHandler.hpp"



    USVTransformHandler::USVTransformHandler(rclcpp::Node::SharedPtr node):
    node_(node),
    heading_(0.0),
    pitch_(0.0),
    roll_(0.0)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        get_pose_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&USVTransformHandler::update_usv_position, this));

    }
    void USVTransformHandler::update_usv_position(){
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_buffer_->lookupTransform(
                "world_ned",
                "usv_ned",
                tf2::TimePointZero,
                tf2::durationFromSec(0.1)); // Timeout
            pose_= transform.transform;
        } catch(const tf2::TransformException &tf_ex){
            RCLCPP_WARN(node_->get_logger(),"Failure to get transform from world to USV: %s", tf_ex.what());
            return;
        }

        tf2::Quaternion quat_tf;
        geometry_msgs::msg::Quaternion quat_msg = pose_.rotation;
        tf2::fromMsg(quat_msg, quat_tf);

        tf2::Matrix3x3 m_rot(quat_tf);
        m_rot.getRPY(roll_, pitch_, heading_);
    }

    geometry_msgs::msg::Transform USVTransformHandler::get_usv_position(){
        return pose_;
    }

    double USVTransformHandler::get_heading(){
        return heading_;
    }
    
