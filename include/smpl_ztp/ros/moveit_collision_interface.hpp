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
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace smpl {
namespace collision{


/// \brief A collision checker that uses MoveIt! to check for collisions. Uses smpl::CollisionChecker interface.
class moveit_collision_interface : public CollisionChecker {

public:
    moveit_collision_interface();
    ~moveit_collision_interface() override = default;

    /// \brief initialize collision checker
    /// \param group_name name of the moveit group
    /// \param grid pointer to the occupancy grid
    bool init(std::string &group_name, OccupancyGrid* grid, std::string planning_frame);

    /// \brief check if state is valid
    /// \param state robot state
    /// \param verbose print verbose output. Default is false.
    /// \return true if valid, false otherwise
    bool isStateValid(const RobotState& state, bool verbose=false) override;

    /// \brief check if state to state interpolation is valid
    /// \param start start state
    /// \param finish finish state
    /// \param verbose print verbose output. Default is false.
    /// \return true if valid, false otherwise
    bool isStateToStateValid(const RobotState& start,
                             const RobotState& finish,
                             bool verbose = false) override;
    /// \brief check if state to state interpolation is valid
    /// \param start start state
    /// \param finish finish state
    /// \param traj interpolated trajectory
    /// \param verbose print verbose output. Default is false.
    /// \return true if valid, false otherwise
    bool isStateToStateValid(const RobotState& start,
                             const RobotState& finish,
                             std::vector<RobotState>& traj,
                             bool verbose = false);

    /// \brief Interpolate a path between two states
    /// \param start start state
    /// \param finish finish state
    /// \param traj interpolated trajectory
    /// \return true if successful, false otherwise
    bool interpolatePath(const RobotState& start,
                         const RobotState& finish,
                         std::vector<RobotState>& traj) override;

    /// \brief Profile a path as a trajectory message
    /// \param start start state
    /// \param finish finish state
    /// \param traj interpolated trajectory
    /// \param trajectory_msg trajectory message
    /// \return true if successful, false otherwise
    bool profileTrajectory(const RobotState& start,
                           const RobotState& finish,
                           std::vector<RobotState>& traj,
                           trajectory_msgs::JointTrajectory &trajectory_msg);

    /// \name Visualization Functions
    /// @{
    /// TODO: FIX this function
    auto getCollisionModelVisualization(const RobotState& vals) -> std::vector<visual::Marker> override;

    /// \brief get collision world visualization
    /// \return marker array
    auto getCollisionWorldVisualization() const -> visualization_msgs::MarkerArray override;

    auto getBoundingBoxVisualization() const -> visual::Marker;

    auto getDistanceFieldVisualization() const -> visual::Marker;

    auto getOccupiedVoxelsVisualization() const -> visual::Marker;
    /// @}

    /// \name Collision Object Functions
    /// \brief insert object
    /// \param object collision object
    /// \return true if successful, false otherwise
    bool insertObject(const smpl::collision::CollisionObject* object) override;

    /// \brief remove object
    /// \param object collision object
    /// \return true if successful, false otherwise
    bool removeObject(const smpl::collision::CollisionObject* object) override;

    /// \brief remove all objects
    void removeAllObjects();

    bool insertShapes(const smpl::collision::CollisionObject* object);

    bool removeShapes(const smpl::collision::CollisionObject* object);

    bool moveShapes(const smpl::collision::CollisionObject* object);

    ///@}

    /// \name Attached Objects Functions
    /// @{
    /// \brief attach object to robot
    /// \param id object id
    /// \param shapes object shapes
    /// \param transforms object transforms
    /// \param link_name link name
    /// \return true if successful, false otherwise
    bool attachObject(
            const std::string& id,
            const std::vector<shapes::ShapeConstPtr>& shapes,
            const Isometry3dVector& transforms,
            const std::string& link_name);

    /// \brief detach object from robot
    /// \param id object id
    /// \return true if successful, false otherwise
    bool detachObject(const std::string& id);
    ///@}

    void setRobotModel(std::unique_ptr<sbpl_interface::MoveItRobotModel> robot_model);

    void getRobotModel(std::unique_ptr<sbpl_interface::MoveItRobotModel> &robot_model);

    bool setJointPosition(const std::string &joint_name, double position);

    bool execute();

    /// \name Utility Functions
    ///@{
    /// \brief convert robot state to moveit robot state
    /// \param state robot state
    /// \param moveit_state moveit robot state
    void convertRobotState(const RobotState& state, moveit::core::RobotState& moveit_state);

    ///@}


    /// \name getters
    ///@{
    auto grid() -> OccupancyGrid* { return m_grid; }
    auto grid() const -> const OccupancyGrid* { return m_grid; }

    /// \brief get reference frame
    /// \return reference frame
    const std::string& getReferenceFrame() const override;

    /// \brief get world collision model
    /// \return world collision model
    auto worldCollisionModel() const -> WorldCollisionModelConstPtr
    { return m_wcm; }

    auto getPlanningSceneInterface() const -> const moveit::planning_interface::PlanningSceneInterface
    {return planning_scene_interface_;}

    ///@}

    /// \name Required Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

protected:

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
    std::shared_ptr<scene::scene_builder> scene_builder_;

    WorldCollisionModelPtr m_wcm;
    OccupancyGrid* m_grid;

    // Active robot joint names
    std::vector<std::string> active_joint_names_;

    // Its not being used currently
    /// @{
    std::string reference_frame_;

    smpl::Isometry3 transform_eigen_;
    /// @}



};
} // namespace collision
} // namespace smpl

#endif //SMPL_ZTP_MOVEIT_COLLISION_INTERFACE_HPP
