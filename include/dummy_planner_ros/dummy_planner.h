/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


using std::string;

#ifndef DUMMY_PLANNER_CPP
#define DUMMY_PLANNER_CPP

namespace dummy_planner {

    class DummyPlanner : public nav_core::BaseGlobalPlanner {
    public:

        DummyPlanner();
        DummyPlanner(std::string , costmap_2d::Costmap2DROS* );

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string , costmap_2d::Costmap2DROS* );
        bool makePlan(const geometry_msgs::PoseStamped& ,
                        const geometry_msgs::PoseStamped& ,
                        std::vector<geometry_msgs::PoseStamped>& );
        
    private:
        bool initialized_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        double min_dist_from_robot_, step_size_;
        base_local_planner::WorldModel* world_model_;

        double footprintCost(double, double, double);
    };


};  
#endif