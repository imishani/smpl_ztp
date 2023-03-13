//
// Created by itamar on 3/12/23.
//

#ifndef SMPL_ZTP_MOVEIT_COLLISION_INTERFACE_HPP
#define SMPL_ZTP_MOVEIT_COLLISION_INTERFACE_HPP


// project includes
#include <smpl/collision_checker.h>
#include "smpl_ztp/planner/moveit_robot_model.h"
#include <scene_builder.hpp>

// ros/moveit includes
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <sbpl_collision_checking/world_collision_model.h>

namespace smpl {
namespace collision{


/// \brief A collision checker that uses MoveIt! to check for collisions. Uses smpl::CollisionChecker interface.
class moveit_collision_interface : public CollisionChecker {

public:
    moveit_collision_interface();
    ~moveit_collision_interface() override = default;

    bool init(std::string &group_name, OccupancyGrid* grid);

//    bool init(const moveit::core::RobotModelConstPtr& robot_model);
    //set verbose to false by default
    bool isStateValid(const RobotState& state, bool verbose=false) override;

    bool isStateToStateValid(const RobotState& start,
                             const RobotState& finish,
                             bool verbose = false) override;

    bool isStateToStateValid(const RobotState& start,
                             const RobotState& finish,
                             std::vector<RobotState>& traj,
                             bool verbose = false);

//    bool isStateToStateValid(const RobotState& start,
//                             const RobotState& finish,
//                             std::vector<RobotState>& traj,
//                             std::vector<Extension>& extensions,
//                             bool verbose = false);

    bool interpolatePath(const RobotState& start,
                         const RobotState& finish,
                         std::vector<RobotState>& traj) override;

    bool profileTrajectory(const RobotState& start,
                           const RobotState& finish,
                           std::vector<RobotState>& traj,
                           trajectory_msgs::JointTrajectory &trajectory_msg);

    auto getCollisionModelVisualization(const RobotState& vals) -> std::vector<visual::Marker> override;

    auto getCollisionWorldVisualization() const -> visualization_msgs::MarkerArray override;

    /// \brief get reference frame
    const std::string& getReferenceFrame() const override;

    /// \brief insert object
    bool insertObject(const smpl::collision::CollisionObject* object) override;

    /// \brief remove object
    bool removeObject(const smpl::collision::CollisionObject* object) override;

    /// \brief remove all objects
    bool removeAllObjects();

    /// \brief convert robot state to moveit robot state
    void convertRobotState(const RobotState& state, moveit::core::RobotState& moveit_state);

    void setRobotModel(std::unique_ptr<sbpl_interface::MoveItRobotModel> robot_model);

    void getRobotModel(std::unique_ptr<sbpl_interface::MoveItRobotModel> &robot_model);

private:

    std::unique_ptr<sbpl_interface::MoveItRobotModel> robot_model_;
    // planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    const moveit::core::JointModelGroup* joint_model_group_;

    std::string group_name_;

    // collision detection
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;

    // Scene builder
    scene::scene_builder scene_builder_;

    WorldCollisionModelPtr m_wcm;
    OccupancyGrid* m_grid;



};
} // namespace collision
} // namespace smpl

#endif //SMPL_ZTP_MOVEIT_COLLISION_INTERFACE_HPP
