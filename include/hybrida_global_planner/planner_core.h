#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#define POT_HIGH 1.0e10        // unassigned cell potential

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <hybrid_astar_global_planner/HybridGlobalPlannerConfig.h>
#include "hybrid_astar/constants.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/collisiondetection.h"
#include "hybrid_astar/dynamicvoronoi.h"
#include "hybrid_astar/algorithm.h"
#include "hybrid_astar/node3d.h"
#include "hybrid_astar/path.h"
#include "hybrid_astar/smoother.h"
#include "hybrid_astar/lookup.h"

namespace hybrid_astar_global_planner {

    class Expander;

    class GridPath;

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

    class HybridGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        HybridGlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        HybridGlobalPlanner(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~HybridGlobalPlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief  Computes the full navigation function for the map given a point in the world to start from
         * @param world_point The point to use for seeding the navigation function
         * @return True if the navigation function was computed successfully, false otherwise
         */
        bool computePotential(const geometry_msgs::Point &world_point);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return The navigation function's value at that point in the world
         */
        double getPointPotential(const geometry_msgs::Point &world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        bool validPointPotential(const geometry_msgs::Point &world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @param tolerance The tolerance on searching around the world_point specified
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        bool validPointPotential(const geometry_msgs::Point &world_point, double tolerance);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

        bool makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D *costmap_;
        nav_msgs::OccupancyGrid::Ptr grid;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_, allow_unknown_, visualize_potential_;

    private:
        void mapToWorld(double mx, double my, double &wx, double &wy);

        bool worldToMap(double wx, double wy, double &mx, double &my);

        void clearRobotCell(const tf::Stamped<tf::Pose> &global_pose, unsigned int mx, unsigned int my);

//        void publishPotential(float *potential);

        void updateGrid();

        nav_msgs::OccupancyGrid::Ptr grid_;
        static char *cost_translation_table_;
        double saved_origin_x_, saved_origin_y_;
        double planner_window_x_, planner_window_y_, default_tolerance_;
        std::string tf_prefix_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        bool publish_potential_;
        ros::Publisher potential_pub_;
        int publish_scale_;

        void outlineMap(unsigned char *costarr, int nx, int ny, unsigned char value);

        unsigned char *cost_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        bool old_navfn_behavior_;
        float convert_offset_;

        dynamic_reconfigure::Server<HybridGlobalPlannerConfig> *dsrv_;

        void reconfigureCB(HybridGlobalPlannerConfig &config, uint32_t level);

        /// The path produced by the hybrid A* algorithm
        HybridAStar::Path path;
        /// The smoother used for optimizing the path
        HybridAStar::Smoother smoother;
        /// The path smoothed and ready for the controller
        HybridAStar::Path smoothedPath = HybridAStar::Path(true);
        /// The voronoi diagram
        HybridAStar::DynamicVoronoi voronoiDiagram;
        /// The collission detection for testing specific configurations
        HybridAStar::CollisionDetection configurationSpace;
        /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
        HybridAStar::Constants::config collisionLookup[HybridAStar::Constants::headings * HybridAStar::Constants::positions];
        /// A lookup of analytical solutions (Dubin's paths)
        float* dubinsLookup = new float [HybridAStar::Constants::headings * HybridAStar::Constants::headings * HybridAStar::Constants::dubinsWidth * HybridAStar::Constants::dubinsWidth];
        /// The visualization used for search visualization
        HybridAStar::Visualize visualization;

    };

} //end namespace hybrid_astar_global_planner

#endif
