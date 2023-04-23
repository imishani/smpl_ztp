////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023, Itamar Mishani
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \file frame_planner.cpp
/// \author Itamar Mishani
/// \date 03/10/2023


// standard includes
#include <cstdlib>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/edge_euclid_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <smpl_ztp/planner/moveit_robot_model.h>
#include <smpl_ztp/ros/planner_interface_ztp.h>
#include <visualization_msgs/MarkerArray.h>
#include "collision_space_scene_moveit.hpp"
#include "pr2_allowed_collision_pairs.h"
#include <smpl/console/console.h>
#include <smpl_ztp/ros/moveit_collision_interface.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


class PickPlaceGripperMachine
{
public:
    // Constructor
    PickPlaceGripperMachine() {
        planning_group_ = "gripper_1";
        ROS_INFO_STREAM("Planning group: " << planning_group_);
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_);
    }

    PickPlaceGripperMachine(std::string& planning_group){
        planning_group_ = std::move(planning_group);
        ROS_INFO_STREAM("Planning group: " << planning_group_);
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_);
    }

    // Destructor
    ~PickPlaceGripperMachine() = default;

    bool openGripper() {
        move_group_->setNamedTarget("open_1");  // &"open_" [ planning_group_.back()]
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(my_plan);
        }
        return success;
    }

    bool closeGripper() {
        move_group_->setNamedTarget("closed_1");  // &"close_" [ planning_group_.back()]
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(my_plan);
        }
        return success;
    }

    void pick(geometry_msgs::Pose& pre_grasp,
              geometry_msgs::Pose& grasp,
              std::string& frame_id,
              std::string& object_id){

        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        grasps[0].grasp_pose.pose = grasp;
        grasps[0].grasp_pose.header.frame_id = frame_id;

        grasps[0].pre_grasp_approach.direction.header.frame_id = frame_id;
        grasps[0].pre_grasp_approach.min_distance = 0.095;
        grasps[0].pre_grasp_approach.desired_distance = 0.115;
        // Normalize grasp - pre_grasp vector
        auto norm = sqrt(pow(grasp.position.x - pre_grasp.position.x, 2) +
                         pow(grasp.position.y - pre_grasp.position.y, 2) +
                         pow(grasp.position.z - pre_grasp.position.z, 2));
//        grasps[0].pre_grasp_approach.direction.vector.x = (grasp.position.x - pre_grasp.position.x)/norm;
//        grasps[0].pre_grasp_approach.direction.vector.y = (grasp.position.y - pre_grasp.position.y)/norm;
//        grasps[0].pre_grasp_approach.direction.vector.z = (grasp.position.z - pre_grasp.position.z)/norm;
        grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
        ROS_INFO_STREAM("Pre-grasp approach direction: " << grasps[0].pre_grasp_approach.direction.vector.x << " "
                                                         << grasps[0].pre_grasp_approach.direction.vector.y << " "
                                                         << grasps[0].pre_grasp_approach.direction.vector.z);


        grasps[0].post_grasp_retreat.direction.header.frame_id = frame_id;
        // oposite direction
//        grasps[0].post_grasp_retreat.direction.vector.x = -grasps[0].pre_grasp_approach.direction.vector.x;
//        grasps[0].post_grasp_retreat.direction.vector.y = -grasps[0].pre_grasp_approach.direction.vector.y;
//        grasps[0].post_grasp_retreat.direction.vector.z = -grasps[0].pre_grasp_approach.direction.vector.z;
        grasps[0].post_grasp_retreat.direction.vector.z = -1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.095;
        grasps[0].post_grasp_retreat.desired_distance = 0.115;

        // set pre-grasp posture based on Open
        grasps[0].pre_grasp_posture.joint_names.resize(1);
        std::string joint_name = std::string("arm_") + planning_group_.back() + std::string("robotiq_85_left_knuckle_joint");
        ROS_INFO_STREAM("Joint name: " << joint_name);
        grasps[0].pre_grasp_posture.joint_names[0] = joint_name;
        grasps[0].pre_grasp_posture.points.resize(1);
        grasps[0].pre_grasp_posture.points[0].positions.resize(1);
        grasps[0].pre_grasp_posture.points[0].positions[0] = move_group_->getNamedTargetValues("open_1")[joint_name];   //&"open_" [ planning_group_.back()]
        ROS_INFO_STREAM("Pre-grasp posture: " << grasps[0].pre_grasp_posture.points[0].positions[0]);
        grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

        // set grasp posture based on Close
        grasps[0].grasp_posture.joint_names.resize(1);
        grasps[0].grasp_posture.joint_names[0] = joint_name;
        grasps[0].grasp_posture.points.resize(1);
        grasps[0].grasp_posture.points[0].positions.resize(1);
        grasps[0].grasp_posture.points[0].positions[0] = move_group_->getNamedTargetValues("closed_1")[joint_name]; // &"close_" [ planning_group_.back()]
        ROS_INFO_STREAM("Grasp posture: " << grasps[0].grasp_posture.points[0].positions[0]);
        grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

//        move_group_.setSupportSurfaceName("table");
        ROS_INFO("Picking object %s", object_id.c_str());
        ROS_INFO_STREAM("Move group: " << move_group_->getName());
//        move_group_->pick(object_id, grasps);
    }

    void place(
            const std::string& object_id,
            const std::vector<double>& pose,
            const std::string& frame_id)
    {
        ROS_INFO("Placing object %s", object_id.c_str());
        geometry_msgs::PoseStamped place_pose;
        place_pose.header.frame_id = frame_id;
        place_pose.pose.position.x = pose[0];
        place_pose.pose.position.y = pose[1];
        place_pose.pose.position.z = pose[2];
        place_pose.pose.orientation.x = pose[3];
        place_pose.pose.orientation.y = pose[4];
        place_pose.pose.orientation.z = pose[5];
        place_pose.pose.orientation.w = pose[6];

        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);
        place_location[0].place_pose = place_pose;
        place_location[0].pre_place_approach.direction.header.frame_id = frame_id;
        place_location[0].pre_place_approach.min_distance = 0.095;
        place_location[0].pre_place_approach.desired_distance = 0.115;
        place_location[0].pre_place_approach.direction.vector.z = 1.0;

        place_location[0].post_place_retreat.direction.header.frame_id = frame_id;
        place_location[0].post_place_retreat.direction.vector.z = -1.0;
        place_location[0].post_place_retreat.min_distance = 0.095;
        place_location[0].post_place_retreat.desired_distance = 0.115;

        place_location[0].post_place_posture.joint_names.resize(1);
        std::string joint_name = std::string("arm_") + planning_group_.back() + std::string("robotiq_85_left_knuckle_joint");
        place_location[0].post_place_posture.joint_names[0] = joint_name;
        place_location[0].post_place_posture.points.resize(1);
        place_location[0].post_place_posture.points[0].positions.resize(1);
        place_location[0].post_place_posture.points[0].positions[0] = move_group_->getNamedTargetValues("open_1")[joint_name];   //&"open_" [ planning_group_.back()]
        place_location[0].post_place_posture.points[0].time_from_start = ros::Duration(0.5);

//        move_group_->place(object_id, place_location);
    }

    void brutalPick(moveit::planning_interface::MoveGroupInterface& group_,
                    geometry_msgs::PoseStamped& grasp_pose_,
                    std::string& object_name,
                    moveit::planning_interface::MoveGroupInterface::Plan& plan_,
                    smpl::PlannerInterface& planner_){
        group_.clearPathConstraints();
        group_.clearPoseTargets();
        // Plan to grasp pose
        ROS_INFO_STREAM("Plan to grasp pose: " << grasp_pose_);
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp;
        group_.setPoseTarget(grasp_pose_);
        bool success = (group_.plan(plan_grasp) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");
        group_.execute(plan_grasp);

        ROS_INFO("Reached grasp pose");

//        bool succ = group_.attachObject(object_name, "arm_1TCP");
//        ROS_INFO_STREAM("Attach object: " << std::boolalpha << succ << std::endl);
        group_.setStartStateToCurrentState();

        group_.clearPathConstraints();
        group_.clearPoseTargets();
        // Reverse both plans
        ROS_INFO("Reverse both plans");

        std::reverse(plan_.trajectory_.joint_trajectory.points.begin(),
                     plan_.trajectory_.joint_trajectory.points.end());
        std::reverse(plan_grasp.trajectory_.joint_trajectory.points.begin(),
                     plan_grasp.trajectory_.joint_trajectory.points.end());

        // Delete all calculated velocities and accelerations
        for (int i = 0; i < plan_.trajectory_.joint_trajectory.points.size(); i++){
            plan_.trajectory_.joint_trajectory.points[i].velocities.clear();
            plan_.trajectory_.joint_trajectory.points[i].accelerations.clear();
        }
        for (int i = 0; i < plan_grasp.trajectory_.joint_trajectory.points.size(); i++){
            plan_grasp.trajectory_.joint_trajectory.points[i].velocities.clear();
            plan_grasp.trajectory_.joint_trajectory.points[i].accelerations.clear();
        }

        // Concatenate the two plans where the reversed plan_grasp is the first one
        ROS_INFO("Concatenate the two plans");
        ROS_INFO_STREAM("Plan size: " << plan_.trajectory_.joint_trajectory.points.size());
        plan_.trajectory_.joint_trajectory.points.insert(plan_.trajectory_.joint_trajectory.points.begin(),
                                                         plan_grasp.trajectory_.joint_trajectory.points.begin(),
                                                         plan_grasp.trajectory_.joint_trajectory.points.end());

//        moveit_msgs::RobotTrajectory trajectory;
//        trajectory.joint_trajectory.points.resize(plan_.trajectory_.joint_trajectory.points.size());
//        for (int i = 0; i < plan_.trajectory_.joint_trajectory.points.size(); i++){
//            trajectory.joint_trajectory.points[i].positions = plan_.trajectory_.joint_trajectory.points[i].positions;
//        }
//        trajectory.joint_trajectory.header.seq = 0;
//        trajectory.joint_trajectory.header.stamp = ros::Time::now();
//        trajectory.joint_trajectory.header.frame_id = "world";
        ROS_INFO_STREAM("Plan size: " << plan_.trajectory_.joint_trajectory.points.size());
        // Re-profile the trajectory
        ROS_INFO("Re-profile the trajectory");
        moveit::planning_interface::MoveGroupInterface::Plan concat_plan;
        concat_plan.trajectory_.joint_trajectory = plan_.trajectory_.joint_trajectory;
        // Print the trajectory
//        ROS_INFO("Print the trajectory");
//        for (auto& point_ : concat_plan.trajectory_.joint_trajectory.points) {
//            ROS_INFO_STREAM(point_);
//        }
        ROS_INFO("Profile the trajectory");
        planner_.profilePath(concat_plan.trajectory_.joint_trajectory, true);
        // Execute the trajectory
        ROS_INFO("Execute the trajectory");
        group_.execute(concat_plan);
    }

    void brutalPlace(moveit::planning_interface::MoveGroupInterface& group_,
                     geometry_msgs::PoseStamped& place_pose_,
                     std::string& object_name,
                     moveit::planning_interface::MoveGroupInterface::Plan& plan_,
                     smpl::PlannerInterface& planner_){
        // Clean the group_ plan
        group_.clearPathConstraints();
        group_.clearPoseTargets();

        // Plan to place pose
        ROS_INFO_STREAM("Plan to place pose: " << place_pose_);
        moveit::planning_interface::MoveGroupInterface::Plan plan_place;
        group_.setPoseTarget(place_pose_);
        bool success = (group_.plan(plan_place) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");
        group_.execute(plan_place);

        ROS_INFO("Reached place pose");

        bool succ = group_.detachObject(object_name);
        ROS_INFO_STREAM("Detach object: " << std::boolalpha << succ << std::endl);
        group_.setStartStateToCurrentState();
        group_.clearPathConstraints();
        group_.clearPoseTargets();

        // detach all objects

        // Reverse both plans
        ROS_INFO("Reverse both plans");
        std::reverse(plan_.trajectory_.joint_trajectory.points.begin(),
                     plan_.trajectory_.joint_trajectory.points.end());
        std::reverse(plan_place.trajectory_.joint_trajectory.points.begin(),
                     plan_place.trajectory_.joint_trajectory.points.end());

        // Concatenate the two plans where the reversed plan_place is the first one
        ROS_INFO("Concatenate the two plans");
        plan_.trajectory_.joint_trajectory.points.insert(plan_.trajectory_.joint_trajectory.points.begin(),
                                                         plan_place.trajectory_.joint_trajectory.points.begin(),
                                                         plan_place.trajectory_.joint_trajectory.points.end());

        // Re-profile the trajectory
        ROS_INFO("Re-profile the trajectory");
        moveit::planning_interface::MoveGroupInterface::Plan concat_plan;
        concat_plan.trajectory_.joint_trajectory = plan_.trajectory_.joint_trajectory;
        // Print the trajectory
//        ROS_INFO("Print the trajectory");
//        for (auto& point_ : concat_plan.trajectory_.joint_trajectory.points) {
//            ROS_INFO_STREAM(point_);
//        }
        ROS_INFO("Profile the trajectory");
        planner_.profilePath(concat_plan.trajectory_.joint_trajectory, true);
        // Execute the trajectory
        ROS_INFO("Execute the trajectory");
        group_.execute(concat_plan);
    }

private:
    std::string planning_group_;
    // define a move group field
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

void FillGoalConstraint(
        const std::vector<double>& pose,
        std::string frame_id,
        moveit_msgs::Constraints& goals)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

    Eigen::Quaterniond q;
    SMPL_INFO_STREAM("Roll, Pitch, Yaw: " << pose[3] << " " << pose[4] << " " << pose[5]);
    smpl::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
        const geometry_msgs::Pose& pose,
        std::vector<double>& dims,
        const std::string& frame_id,
        const std::string& id)
-> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

auto GetCollisionCubes(
        std::vector<std::vector<double>>& objects,
        std::vector<std::string>& object_ids,
        const std::string& frame_id)
-> std::vector<moveit_msgs::CollisionObject>
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(
        const std::string& filename,
        const std::string& frame_id)
-> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == nullptr) { //NULL
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.emplace_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

/// \brief Get the collision objects from the planning scene
/// \param planning_scene_interface
/// \return a map of collision objects (key is the object id)
auto GetCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
-> std::vector<moveit_msgs::CollisionObject>{
    auto collision_objects = planning_scene_interface.getObjects();
    std::vector<moveit_msgs::CollisionObject> objs;
    for (auto& obj : collision_objects) {
        objs.push_back(obj.second);
    }
    return objs;
}

bool ReadInitialConfiguration(
        ros::NodeHandle& nh,
        moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

auto SetupMoveItRobotModel(const std::string& urdf, const RobotModelConfig &config,
                           std::shared_ptr<moveit::core::RobotModel>& robot_model,
                           const std::string& group_name = "manipulator_1") // TODO: delete this. We have the group name in config
-> std::unique_ptr<sbpl_interface::MoveItRobotModel>
{
    std::unique_ptr<sbpl_interface::MoveItRobotModel> rm;

    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
        return rm;
    }

    ROS_INFO("Construct Generic MoveIt Robot Model");
    rm.reset(new sbpl_interface::MoveItRobotModel);

    ROS_INFO("Initialize UR10 MoveIt Robot Model");

//    std::vector<std::string> redundant_joints; UR10 has no redundant joints
    auto joint_group = robot_model->getJointModelGroup(group_name);

    if (!rm->init(
            robot_model,
            group_name)) // right_arm
    {
        ROS_ERROR("Failed to initialize robot model.");
        rm.reset();
        return std::move(rm);
    }

    if (!rm->setPlanningLink(config.chain_tip_link)) {
        ROS_ERROR("Failed to set planning link to '%s'", config.chain_tip_link.c_str());
        rm.reset();
        return std::move(rm);
    }

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    rm->setPlanningScene(planning_scene);
    rm->setPlanningFrame(planning_scene->getPlanningFrame());

    return std::move(rm);
}

void initAllowedCollisionsPR2(smpl::collision::CollisionSpace &cspace)
{
    smpl::collision::AllowedCollisionMatrix acm;
    for (auto& pair : PR2AllowedCollisionPairs) {
        acm.setEntry(pair.first, pair.second, true);
    }
    cspace.setAllowedCollisionMatrix(acm);
}


void SetJoints(moveit_msgs::RobotState &state,
               smpl::collision::CollisionSpace &cspace){
    SMPL_INFO("Setting CollisionSpace joints to initial state..");
    int j {0};
    for (auto& joint : state.joint_state.name) {
        SMPL_INFO("Joint: %s", joint.c_str());
        cspace.setJointPosition(joint, state.joint_state.position[j]);
        cspace.m_rcs->setJointVarPosition(joint, state.joint_state.position[j]);
        ++j;
    }
}

void convertFromRelativePose(const geometry_msgs::Pose &pose,
                             const geometry_msgs::Pose &reference,
                             std::vector<double> &result)
{
    tf::Transform tf_pose;
    tf::poseMsgToTF(pose, tf_pose);
    // Print rf_pose
    tf::Quaternion q_pose(tf_pose.getRotation());
    tf::Vector3 v_pose(tf_pose.getOrigin());
    SMPL_INFO("Pose: %f %f %f %f %f %f %f", v_pose.getX(), v_pose.getY(), v_pose.getZ(), q_pose.getX(), q_pose.getY(), q_pose.getZ(), q_pose.getW());

    tf::Transform tf_reference;
    tf::poseMsgToTF(reference, tf_reference);
    // Print rf_reference
    tf::Quaternion q_reference(tf_reference.getRotation());
    tf::Vector3 v_reference(tf_reference.getOrigin());
    SMPL_INFO("Reference: %f %f %f %f %f %f %f", v_reference.getX(), v_reference.getY(), v_reference.getZ(), q_reference.getX(), q_reference.getY(), q_reference.getZ(), q_reference.getW());


    tf::Transform tf_result = tf_reference * tf_pose;
    // Print rf_result
    tf::Quaternion q_result(tf_result.getRotation());
    tf::Vector3 v_result(tf_result.getOrigin());
    SMPL_INFO("Result: %f %f %f %f %f %f %f", v_result.getX(), v_result.getY(), v_result.getZ(), q_result.getX(), q_result.getY(), q_result.getZ(), q_result.getW());

    geometry_msgs::Pose result_gp;
    tf::poseTFToMsg(tf_result, result_gp);

    result.resize(6);
    result[0] = result_gp.position.x;
    result[1] = result_gp.position.y;
    result[2] = result_gp.position.z;
    // Convert from quaternion to rpy
    tf::Quaternion q(result_gp.orientation.x, result_gp.orientation.y, result_gp.orientation.z, result_gp.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(result[3], result[4], result[5]);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "frame_ztp");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

//    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//        ros::console::notifyLoggerLevelsChanged();
//    }

    ros::AsyncSpinner spinner(1);
    spinner.start();


    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    // let publishers set up
    ros::Duration(1.0).sleep();

    /////////////////
    // Robot Model //
    /////////////////
    ROS_INFO("Load common parameters");

    // Robot description required to initialize collision checker and robot
    // model...
    const char *robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string urdf;
    if (!nh.getParam(robot_description_param, urdf)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }
    // Instantiate a RobotModelLoader pointer object which will look up the robot description
    // on the parameter server and construct a RobotModel object for us to use.
    auto robot_loader = boost::make_shared<robot_model_loader::RobotModelLoader>(
            "robot_description", true);
    auto robot_model = robot_loader->getModel();
    if (!robot_model) {
        ROS_ERROR("Robot model is null");
        return 1;
    }

    // Read arm name from parameter server
    std::string arm_name;
    if (!ph.getParam("arm", arm_name)) {
        ROS_ERROR("Failed to retrieve param 'arm' from the param server");
        return 1;
    }

    auto rm = SetupMoveItRobotModel(urdf, robot_config, robot_model, arm_name);

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    const double df_size_x = 3.0;
    const double df_size_y = 3.0;
    const double df_size_z = 3.0;
    const double df_res = 0.02;
    const double df_origin_x = -0.50;
    const double df_origin_y = -1.5;
    const double df_origin_z = 0.0;
    const double max_distance = 1.8;

//    typedef smpl::EdgeEuclidDistanceMap DistanceMapType;
    typedef smpl::EuclidDistanceMap DistanceMapType;
//    typedef smpl::PropagationDistanceField DistanceMapType;

    ROS_INFO("Create distance map");
    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    ROS_INFO("Create grid");
    auto ref_counted = false;
    smpl::OccupancyGrid grid(df, ref_counted);

    // everyone needs to know the name of the planning frame for reasons...
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ROS_INFO("Planning Frame: %s", planning_frame.c_str());

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    ///////////////////////
    // Collision Checker //
    ///////////////////////
    ROS_INFO("Create collision checker");
    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    CollisionSpaceSceneMoveit scene;

    ROS_INFO("Create moveit collision checker");
    smpl::collision::moveit_collision_interface cc;
    cc.init(robot_config.group_name, &grid, planning_frame);

    //    /////////////////
//    // Scene Setup //
//    /////////////////
//
    scene.SetCollisionSpace(&cc);
    // read in collision objects from file and add to the scene
    auto planning_scene_int = cc.getPlanningSceneInterface();
    auto objects = GetCollisionObjects(planning_scene_int);

    // Now, when we read the collision objects from the scene we dont need to process them
    //@{
    //    for (auto& object : objects) {
    //      scene.ProcessCollisionObjectMsg(object);
    //}
    //@}

    // read in collision objects from file and add to the scene. Need to be done using scene_builder
    // read in start state from file and update the scene
    ROS_INFO("Read start state from file and update the scene");
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }


//    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

    ///////////////////
    // Planner Setup //
    ///////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
        ROS_ERROR("Failed to read planner config");
        return 1;
    }

    smpl::PlannerInterface planner(rm.get(), &cc, &grid);

    smpl::PlanningParams params;

    params.addParam("discretization", planning_config.discretization);
    params.addParam("mprim_filename", planning_config.mprim_filename);
    params.addParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim);
    params.addParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim);
    params.addParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim);
    params.addParam("use_short_dist_mprims", planning_config.use_short_dist_mprims);
    params.addParam("xyz_snap_dist_thresh", planning_config.xyz_snap_dist_thresh);
    params.addParam("rpy_snap_dist_thresh", planning_config.rpy_snap_dist_thresh);
    params.addParam("xyzrpy_snap_dist_thresh", planning_config.xyzrpy_snap_dist_thresh);
    params.addParam("short_dist_mprims_thresh", planning_config.short_dist_mprims_thresh);
    params.addParam("bfs_inflation_radius", 0.02);
    params.addParam("repair_time", 5.0);

    params.addParam("epsilon", 100.0);

    if (!planner.init(params)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return 1;
    }

    //////////////
    // Planning //
    //////////////
    // check if we need to pick (grasp)
    std::vector<double> place(6, 0);
    ph.param("goal/x", place[0], 0.0);
    ph.param("goal/y", place[1], 0.0);
    ph.param("goal/z", place[2], 0.0);
    ph.param("goal/roll", place[3], 0.0);
    ph.param("goal/pitch", place[4], 0.0);
    ph.param("goal/yaw", place[5], 0.0);

    bool pick;
    ph.param("pick", pick, false);
    std::vector<double> goal(6, 0);
    std::string object_name;
    geometry_msgs::Pose object_pose;
    if (pick){
        ROS_INFO("Looking for a pick object..");
        if (!ph.getParam("grasp_object/name", object_name)) {
            ROS_ERROR("Failed to retrieve param 'object_name' from the param server");
            return 1;
        }
        ROS_INFO("Object name: %s", object_name.c_str());

        // Look for the object in the objects vector and get the pose
        for (auto& object : objects) {
            if (object.id == object_name){
                object_pose.position.x = object.pose.position.x;
                object_pose.position.y = object.pose.position.y;
                object_pose.position.z = object.pose.position.z;
                object_pose.orientation.x = object.pose.orientation.x;
                object_pose.orientation.y = object.pose.orientation.y;
                object_pose.orientation.z = object.pose.orientation.z;
                object_pose.orientation.w = object.pose.orientation.w;
                break;
            }
        }
        // Get the relative pose from param server
        std::vector<double> rel_pose(7, 0);
        ph.param("grasp_object/relative_pose/position/x", rel_pose[0], 0.0);
        ph.param("grasp_object/relative_pose/position/y", rel_pose[1], 0.0);
        ph.param("grasp_object/relative_pose/position/z", rel_pose[2], 0.0);
        ph.param("grasp_object/relative_pose/orientation/x", rel_pose[3], 0.0);
        ph.param("grasp_object/relative_pose/orientation/y", rel_pose[4], 0.0);
        ph.param("grasp_object/relative_pose/orientation/z", rel_pose[5], 0.0);
        ph.param("grasp_object/relative_pose/orientation/w", rel_pose[6], 0.0);
        // Convert to geometry_msgs::Pose
        geometry_msgs::Pose rel_pose_msg;
        rel_pose_msg.position.x = rel_pose[0];
        rel_pose_msg.position.y = rel_pose[1];
        rel_pose_msg.position.z = rel_pose[2];
        rel_pose_msg.orientation.x = rel_pose[3];
        rel_pose_msg.orientation.y = rel_pose[4];
        rel_pose_msg.orientation.z = rel_pose[5];
        rel_pose_msg.orientation.w = rel_pose[6];

        // get the object pose
        convertFromRelativePose(rel_pose_msg, object_pose, goal);
    }
    else {
        goal = place;
    }


    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    req.allowed_planning_time = 60.0;
    req.goal_constraints.resize(1);
    FillGoalConstraint(goal, planning_frame, req.goal_constraints[0]);

    req.group_name = robot_config.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;
//    req.path_constraints;
    req.planner_id = "arastar.workspace_distance.workspace"; //"arastar.bfs.manip";
    req.start_state = start_state;
//    req.trajectory_constraints;
//    req.workspace_parameters;

    // plan
    ROS_INFO("Calling solve...");
    moveit_msgs::PlanningScene planning_scene; // Is this necessary? If so, maybe I should use different name
    planning_scene.robot_state = start_state;

    bool query = false;
    if (!ph.getParam("query", query)) {
        ROS_INFO_STREAM(ph.getNamespace() << "query");
        ROS_ERROR("Failed to read 'query' from the param server");
        return 1;
    }

    bool random_query = false;
    if (!ph.getParam("random_query", random_query)) {
        ROS_INFO_STREAM(ph.getNamespace() << "random_query");
        ROS_ERROR("Failed to read 'random_query' from the param server");
        return 1;
    }
    if (pick){
        if (!planner.solveZero(planning_scene, req, res, query, random_query)) {
            ROS_ERROR("Failed to plan.");
            return 1;
        }
    }

    bool place_query;
    ph.param("place",place_query, false);
    moveit_msgs::MotionPlanRequest req_place;
    moveit_msgs::MotionPlanResponse res_place;
    if (place_query){
        req_place.allowed_planning_time = 60.0;
        req_place.goal_constraints.resize(1);
        FillGoalConstraint(place, planning_frame, req_place.goal_constraints[0]);

        req_place.group_name = robot_config.group_name;
        req_place.max_acceleration_scaling_factor = 1.0;
        req_place.max_velocity_scaling_factor = 1.0;
        req_place.num_planning_attempts = 1;
        req_place.planner_id = "arastar.workspace_distance.workspace";
        req_place.start_state = start_state;
        if (!planner.solveZero(planning_scene, req_place, res_place, query, random_query)) {
            ROS_ERROR("Failed to plan.");
            return 1;
        }
    }
    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////
#if 0
    std::map<std::string, double> planning_stats = planner.getPlannerStats();

    ROS_INFO("Planning statistics");
    for (const auto& entry : planning_stats) {
        ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
    }
#endif

    if (query) {
        ROS_INFO("Execute path");
        std::string gripper_group;
        ph.param("gripper", gripper_group, std::string("gripper_1"));
        PickPlaceGripperMachine pick_place_gripper_machine {gripper_group};

        moveit::planning_interface::MoveGroupInterface move_group(robot_config.group_name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (pick){
            // Execute the trajectory
            plan.trajectory_ = res.trajectory;

            move_group.execute(plan);
            ROS_INFO("Reached Pre-grasped pose");

            // pre grasp from trajectory (last point)
            auto& point = res.trajectory.joint_trajectory.points.back();
            // cast to geometry_msgs::Pose
            geometry_msgs::Pose pose;
            pose.position.x = point.positions[0];
            pose.position.y = point.positions[1];
            pose.position.z = point.positions[2];
            pose.orientation.x = point.positions[3];
            pose.orientation.y = point.positions[4];
            pose.orientation.z = point.positions[5];
            pose.orientation.w = point.positions[6];

            ROS_INFO("Ready to grasp");
            // set grasp pose as the object position and the orientation of the last point
            // get current pose of the arm
            geometry_msgs::PoseStamped grasp_pose = move_group.getCurrentPose();
            grasp_pose.pose.position.z -= 0.12;

            // TODO: Fix this!!! I want to use the classic method of picking anf placing
//        pick_place_gripper_machine.pick(pose,
//                                        grasp_pose,
//                                        planning_frame,
//                                        object_name);

            pick_place_gripper_machine.brutalPick(move_group,
                                                  grasp_pose,
                                                  object_name,
                                                  plan,
                                                  planner);

            ROS_INFO("Grasped object and returned to start pose");
        }

        if (place_query) {
            ROS_INFO("Ready to place");
            moveit::planning_interface::MoveGroupInterface::Plan plan_place;
            plan_place.trajectory_ = res_place.trajectory;
            // Execute the trajectory without
            move_group.execute(plan_place);
            ROS_INFO("Reached Pre-placed pose");
            ROS_INFO("Ready to place");

            geometry_msgs::PoseStamped place_pose = move_group.getCurrentPose();
            place_pose.pose.position.z -= 0.05;

            pick_place_gripper_machine.brutalPlace(move_group,
                                                   place_pose,
                                                   object_name,
                                                   plan_place,
                                                   planner);
        }
        ROS_INFO("Placed object and returned to start pose");
    }
//    spinner.stop();
    ros::shutdown();
    // Avoid segfaults
    ros::waitForShutdown();

    return 0;
}
