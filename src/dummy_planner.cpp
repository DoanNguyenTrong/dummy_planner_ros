#include <pluginlib/class_list_macros.h>
#include "dummy_planner.h"


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dummy_planner::DummyPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace dummy_planner {

DummyPlanner::DummyPlanner ():
    initialized_(false),
    costmap_ros_(NULL){}

DummyPlanner::DummyPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
    costmap_ros_(NULL),
    initialized_(false)
    {
    initialize(name, costmap_ros);
}


void DummyPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if (!initialized_){
        costmap_ros_ = costmap_ros;
        costmap_        = costmap_ros_->getCostmap();
        
        ros::NodeHandle private_nh("~/"+name);
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        world_model_ = new base_local_planner::CostmapModel(*costmap_);
        initialized_ = true;

    }
    else{
        ROS_WARN("DummyPlanner has already been initialized !!!");
    }
}


double DummyPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if (!initialized_){
        ROS_ERROR("DummyPlanner not initialized yet!");
        return -1.;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    if (footprint.size() < 3){
        return -1.;
    }

    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;

}



bool DummyPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal,  
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    if (!initialized_){
        ROS_ERROR("DummyPlanner has not been initialized, call initialize() instead");
        return false;
    }
    ROS_DEBUG("Starting point: {%.3f, %.3f}, goal: {%.3f, %.3f}", 
                start.pose.position.x, start.pose.position.y,
                goal.pose.position.x, goal.pose.position.y);
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
        ROS_ERROR("[DummyPlanner::makePlan] Incorrect frame! costmap_frame: %s, goal_frame: %s",
                    costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }
    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
    
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x= start.pose.position.x;
    double start_y= start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;

    double diff_yaw = angles::shortest_angular_distance(start_yaw, goal_yaw);
    

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw= goal_yaw;
    

    bool done = false;
    double scale = 1.0;
    double dScale=0.01;
    while (!done){
        if (scale < 0){
            target_x = start_x;
            target_y = start_y;
            target_yaw=start_yaw;
            ROS_WARN("[DummyPlanner] Cannot find a valid plan");
            break;
        }

        target_x = start_x + scale * diff_x;
        target_y = start_y + scale * diff_y;
        target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

        double footprint_cost = footprintCost(target_x, target_y, target_yaw);
        if (footprint_cost >=0){
            done = true;
        }
        scale -= dScale;
    }

    plan.push_back(start);
    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    return (done);
}
};