//
// Created by itamar on 3/12/23.
//

#include <utility>

#include "smpl_ztp/ros/moveit_collision_interface.hpp"

smpl::collision::moveit_collision_interface::moveit_collision_interface() : CollisionChecker() {
}

bool smpl::collision::moveit_collision_interface::init(std::string &group_name,
                                                       OccupancyGrid* grid,
                                                       std::string planning_frame) {
    group_name_ = group_name;
    ros::AsyncSpinner spinner(1); spinner.start();
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name_);
    // print current joint values
    std::vector<double> joint_group_positions;
    std::cout << move_group_->getCurrentJointValues().data() << std::endl;
    joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(group_name_);
    active_joint_names_ = joint_model_group_->getActiveJointModelNames();

    reference_frame_ = std::move(planning_frame);

    collision_result_.clear();
    collision_request_.group_name = group_name_;

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    // find the transform from the planning frame to the reference frame
    // Not being used currently
    // @{
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    try {
        tf_listener.waitForTransform(move_group_->getPlanningFrame(), reference_frame_, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(move_group_->getPlanningFrame(), reference_frame_, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    // Use tf_eigen to transform
    tf::poseTFToEigen(transform, transform_eigen_);
    // @}

    // initialize scene_builder
    scene_builder_ = std::make_shared<scene::scene_builder>(reference_frame_); // TODO: change to planning frame

    m_grid = grid;
    m_wcm = std::make_shared<WorldCollisionModel>(m_grid);
    spinner.stop();
    return true;
}

bool smpl::collision::moveit_collision_interface::isStateValid(const RobotState& state, bool verbose){
    // Convert RobotState to moveit_msgs::RobotState
    moveit_msgs::RobotState robot_state_msg;
    robot_state_msg.joint_state.name = active_joint_names_;
    // Express the state in the planning frame using transform_eigen_
    robot_state_msg.joint_state.position = state;

    return planning_scene_monitor_->getPlanningScene()->isStateValid(robot_state_msg, group_name_, verbose);
}

bool smpl::collision::moveit_collision_interface::isStateToStateValid(const RobotState& start, const RobotState& finish, bool verbose) {
    // Convert RobotState to moveit_msgs::RobotState
    // Use moveit to check for collisions for planning group between two states:
    std::vector<RobotState> traj_;
    if (!interpolatePath(start, finish, traj_)) {
        return false;
    }
    return isStateToStateValid(start, finish, traj_, verbose);
}

bool smpl::collision::moveit_collision_interface::isStateToStateValid(const RobotState& start, const RobotState& finish, std::vector<RobotState>& traj, bool verbose) {
    for (auto &state : traj) {
        if (!isStateValid(state, verbose)) {
            return false;
        }
    }
    return true;
}

bool smpl::collision::moveit_collision_interface::interpolatePath(const smpl::RobotState &start,
                                                                  const smpl::RobotState &finish,
                                                                  std::vector<RobotState> &traj) {
    assert((start.size() == active_joint_names_.size()) &&
    (finish.size() == active_joint_names_.size()));

    // check if start and finish are valid
    if (!isStateValid(start, false) || !isStateValid(finish, false)) {
        return false;
    }
    // Linear interpolation between start and finish
    const double res = 0.05;
    // Get the maximum distance between the two states
    double max_dist = 0;
    for (int i = 0; i < start.size(); i++) {
        double dist = std::abs(start[i] - finish[i]);
        if (dist > max_dist) {
            max_dist = dist;
        }
    }
    const int num_steps = max_dist / res;
    traj.resize(num_steps);
    for (int i = 0; i < num_steps; i++) {
        traj[i].resize(start.size());
        for (int j = 0; j < start.size(); j++) {
            traj[i][j] = start[j] + i * (finish[j] - start[j]) / num_steps;
        }
    }
    return true;
}

bool smpl::collision::moveit_collision_interface::profileTrajectory(const smpl::RobotState &start,
                                                                    const smpl::RobotState &finish,
                                                                    std::vector<RobotState> &traj,
                                                                    trajectory_msgs::JointTrajectory &trajectory_msg) {
    assert(start.size() == move_group_->getActiveJoints().size() &&
           finish.size() == move_group_->getActiveJoints().size());

    // check if start and finish are valid
    if (!isStateValid(start, false) || !isStateValid(finish, false)) {
        return false;
    }

    // Proflie trajectory
    // convert traj to moveit_msgs::RobotTrajectory
    moveit_msgs::RobotTrajectory traj_msg;
    traj_msg.joint_trajectory.joint_names = move_group_->getActiveJoints();
    traj_msg.joint_trajectory.points.resize(traj.size());
    for (int i = 0; i < traj.size(); i++) {
        traj_msg.joint_trajectory.points[i].positions = traj[i];
    }

    // get the things ready for iterative parabolic time parameterization
    robot_trajectory::RobotTrajectory robot_trajectory(move_group_->getRobotModel(), group_name_);
    moveit::core::RobotState start_state_moveit(move_group_->getRobotModel());
    convertRobotState(start, start_state_moveit);
    robot_trajectory.setRobotTrajectoryMsg(start_state_moveit, traj_msg);

    // Trajectory processing
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    if (!time_param.computeTimeStamps(robot_trajectory)){
        ROS_ERROR("Failed to compute timestamps for trajectory");
        return false;
    }
    robot_trajectory.getRobotTrajectoryMsg(traj_msg);

    trajectory_msg = traj_msg.joint_trajectory;
    return true;
}

auto smpl::collision::moveit_collision_interface::getCollisionModelVisualization(const RobotState &vals) -> std::vector<visual::Marker> {
    std::vector<visual::Marker> markers;
    return markers;
}

auto smpl::collision::moveit_collision_interface::getCollisionWorldVisualization() const -> visualization_msgs::MarkerArray {
    return m_wcm->getCollisionWorldVisualization();
}

auto smpl::collision::moveit_collision_interface::getBoundingBoxVisualization() const -> visual::Marker
{
    return m_grid->getBoundingBoxVisualization();
}

auto smpl::collision::moveit_collision_interface::getDistanceFieldVisualization() const -> visual::Marker
{
    return m_grid->getDistanceFieldVisualization();
}

auto smpl::collision::moveit_collision_interface::getOccupiedVoxelsVisualization() const -> visual::Marker
{
    return m_grid->getOccupiedVoxelsVisualization();
}

const std::string &smpl::collision::moveit_collision_interface::getReferenceFrame() const {
    return move_group_->getPlanningFrame();
}


void smpl::collision::moveit_collision_interface::convertRobotState(const smpl::RobotState &state,
                                                                    moveit::core::RobotState &moveit_state) {
    moveit_state.setJointGroupPositions(joint_model_group_, state);
}

void smpl::collision::moveit_collision_interface::setRobotModel(
        std::unique_ptr<sbpl_interface::MoveItRobotModel> robot_model) {
    robot_model_ = std::move(robot_model);
}

void smpl::collision::moveit_collision_interface::getRobotModel(
        std::unique_ptr<sbpl_interface::MoveItRobotModel> &robot_model) {
    robot_model = std::move(robot_model_);
}

bool smpl::collision::moveit_collision_interface::insertObject(const smpl::collision::CollisionObject *object) {

    if (!m_wcm->insertObject(object)) {
        ROS_WARN("Reject insertion of object '%s'. Failed to add to world collision model.", object->id.c_str());
        return false;
    }

    // TODO: add object to moveit world using scene_builder
    return true;
}

bool smpl::collision::moveit_collision_interface::removeObject(const smpl::collision::CollisionObject *object) {
    if (!m_wcm->removeObject(object)) {
        ROS_WARN("Reject insertion of object '%s'. Failed to remove to world collision model.", object->id.c_str());
        return false;
    }
    // TODO: remove object from moveit world using scene_builder

    return true;
}

void smpl::collision::moveit_collision_interface::removeAllObjects() {
    m_wcm->removeAllObjects();

}

bool smpl::collision::moveit_collision_interface::insertShapes(const smpl::collision::CollisionObject* object) {
    return m_wcm->insertShapes(object);
}

bool smpl::collision::moveit_collision_interface::removeShapes(const smpl::collision::CollisionObject* object) {
    return m_wcm->removeShapes(object);
}

bool smpl::collision::moveit_collision_interface::moveShapes(const smpl::collision::CollisionObject *object) {
    return m_wcm->moveShapes(object);
}

bool smpl::collision::moveit_collision_interface::attachObject(const std::string& id,
                                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                                               const Isometry3dVector& transforms,
                                                               const std::string& link_name){
    move_group_->attachObject(id, link_name);
}

bool smpl::collision::moveit_collision_interface::detachObject(const std::string& id){
    move_group_->detachObject(id);
}

smpl::Extension *smpl::collision::moveit_collision_interface::getExtension(size_t class_code) {
    if (class_code == GetClassCode<CollisionChecker>()) {
        return this;
    }
    return nullptr;
}

bool smpl::collision::moveit_collision_interface::setJointPosition(const std::string &joint_name, double position) {
    return move_group_->setJointValueTarget(joint_name, position);
}

bool smpl::collision::moveit_collision_interface::execute() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!move_group_->plan(plan)) {
        ROS_ERROR("Failed to plan");
        return false;
    }
    if (!move_group_->execute(plan)) {
        ROS_ERROR("Failed to execute");
        return false;
    }
    return true;
}




