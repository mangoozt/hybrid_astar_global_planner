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
 * Author:
 *
 *********************************************************************/
#include <hybrida_global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hybrid_astar_global_planner::HybridGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_global_planner {

void HybridGlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

HybridGlobalPlanner::HybridGlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

HybridGlobalPlanner::HybridGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

HybridGlobalPlanner::~HybridGlobalPlanner() {
    if (dsrv_)
        delete dsrv_;
}

void HybridGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void HybridGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
ROS_WARN("Init Hybrid!!!!!!!!!!!!");
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;
        visualization.setFrameID(frame_id);
/*
        if (cost_translation_table_ == NULL)
        {
            cost_translation_table_ = new char[256];

            // special values:
            cost_translation_table_[0] = 0;  // NO obstacle
            cost_translation_table_[253] = 99;  // INSCRIBED obstacle
            cost_translation_table_[254] = 100;  // LETHAL obstacle
            cost_translation_table_[255] = -1;  // UNKNOWN

            // regular cost values scale the range 1 to 252 (inclusive) to fit
            // into 1 to 98 (inclusive).
            for (int i = 1; i < 253; i++)
            {
                cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
            }
        }
*/

        //unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;



        //orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        double costmap_pub_freq;
        private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &HybridGlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<HybridGlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<HybridGlobalPlannerConfig>::CallbackType cb = boost::bind(
                &HybridGlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void HybridGlobalPlanner::reconfigureCB(HybridGlobalPlannerConfig& config, uint32_t level) {
    //planner_->setLethalCost(configuration.lethal_cost);
    //path_maker_->setLethalCost(configuration.lethal_cost);
    //planner_->setNeutralCost(configuration.neutral_cost);
    //planner_->setFactor(configuration.cost_factor);
    //publish_potential_ = configuration.publish_potential;
    //orientation_filter_->setMode(configuration.orientation_mode);
}

void HybridGlobalPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool HybridGlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void HybridGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool HybridGlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    return mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY();

}

bool HybridGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool HybridGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();
    visualization.clear();
    //update the configuration space with the current map
    configurationSpace.updateGrid(costmap_);
    // prepare Voronoi
    unsigned int height = costmap_->getSizeInCellsY();
    unsigned int width = costmap_->getSizeInCellsX();
    bool** binMap;
    binMap = new bool*[width];

    for (unsigned int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

    for (unsigned int x = 0; x < width; ++x) {
        for (unsigned int y = 0; y < height; ++y) {
            binMap[x][y] = costmap_->getCost(x,y)!=costmap_2d::FREE_SPACE;
        }
    }

    voronoiDiagram.initializeMap(width, height, binMap);
    voronoiDiagram.update();
    //voronoiDiagram.visualize();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
/*
    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);

    path_maker_->setSize(nx, ny);
*/
    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
    /*
    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);
    */
// LISTS ALLOCATED ROW MAJOR ORDER

    int depth = HybridAStar::Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    auto * nodes3D = new HybridAStar::Node3D[length]();
    auto * nodes2D = new HybridAStar::Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal_x;
    float y = goal_y;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = HybridAStar::Helper::normalizeHeadingRad(t);
    const HybridAStar::Node3D nGoal(x, y, t, 0, 0, nullptr);
    // _________________________
    // retrieving start position
    x = start_x;
    y = start_y;
    t = tf::getYaw(start.pose.orientation);
    // set theta to a value (0,2PI]
    t = HybridAStar::Helper::normalizeHeadingRad(t);
    HybridAStar::Node3D nStart(x, y, t, 0, 0, nullptr);


    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    std::cout<<"\nwidth:"<<width<<"\n";
    std::cout<<"\nheight:"<<height<<"\n";
    HybridAStar::Node3D* nSolution = HybridAStar::Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    //path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    //path.publishPath();
    //path.publishPathNodes();
    //path.publishPathVehicles();

    //smoothedPath.publishPath();
    //smoothedPath.publishPathNodes();
    //smoothedPath.publishPathVehicles();
    //visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    smoothedPath.dump(plan,frame_id_);

    delete [] nodes3D;
    delete [] nodes2D;
    //publish the plan for visualization purposes

    publishPlan(plan);
    return !plan.empty();
}

void HybridGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool HybridGlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}
/*
void HybridGlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

    void HybridGlobalPlanner::updateGrid() {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
        double resolution = costmap_->getResolution();

        grid_->header.frame_id = frame_id_;
        grid_->header.stamp = ros::Time::now();
        grid_->info.resolution = resolution;

        grid_->info.width = costmap_->getSizeInCellsX();
        grid_->info.height = costmap_->getSizeInCellsY();

        double wx, wy;
        costmap_->mapToWorld(0, 0, wx, wy);
        grid_->info.origin.position.x = wx - resolution / 2;
        grid_->info.origin.position.y = wy - resolution / 2;
        grid_->info.origin.position.z = 0.0;
        grid_->info.origin.orientation.w = 1.0;
        saved_origin_x_ = costmap_->getOriginX();
        saved_origin_y_ = costmap_->getOriginY();

        grid_->data.resize(grid_->info.width * grid_->info.height);

        unsigned char* data = costmap_->getCharMap();
        for (unsigned int i = 0; i < grid_->data.size(); i++)
        {
            grid_->data[i] = cost_translation_table_[ data[ i ]];
        }
        
    }*/
} //end namespace hybrid_astar_global_planner

