////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl_ztp/ros/planner_interface_ztp.h>

// standard includes
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <utility>

// system includes
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl/planners/mhaplanner.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <smpl/angles.h>
#include <smpl/post_processing.h>
#include <smpl/time.h>
#include <smpl/types.h>

#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>

#include <smpl/graph/adaptive_workspace_lattice.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl_ztp/graph/workspace_lattice_zero.h>

#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>
#include <smpl_ztp/heuristic/workspace_dist_heuristic.h>

#include <smpl/search/adaptive_planner.h>
#include <smpl/search/arastar.h>
#include <smpl/search/experience_graph_planner.h>
#include <smpl/search/awastar.h>
#include <smpl_ztp/search/arastar_zero.h>

#include <smpl/stl/memory.h>

#include <smpl/ros/factories.h>

// TODO: Organize includes

namespace smpl {

const char* PI_LOGGER = "simple";

/// @{ Making new factories for the new planners: ("/src/smpl/smpl_ros/include/smpl/ros/factories.h")
auto MakeWorkspaceLatticeZero(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice Zero");

    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    // TODO: What are these magic numbers? make this a parameter
    wsp.R_count = 36; //72;
    wsp.P_count = 18 + 1; //36 + 1;
    wsp.Y_count = 36; //72;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice Zero requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(5.0));

    struct SimpleWorkspaceLatticeZero : public WorkspaceLatticeZero
    {
        SimpleWorkspaceLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleWorkspaceLatticeZero>();
    if (!space->init(robot, checker, wsp, &space->actions)) {
        ROS_ERROR("Failed to initialize Workspace Lattice Zero");
        return nullptr;
    }
    space->actions.m_ik_amp_enabled = false;

    if (!InitSimpleWorkspaceLatticeActions(space.get(), &space->actions)) {
        return nullptr;
    }

    space->setVisualizationFrameId(grid->getReferenceFrame());

    return std::move(space);
}

auto MakeWorkspaceDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<WorkspaceDistHeuristic>();
    if (!h->init(space)) {
        return nullptr;
    }
    return std::move(h);
};

auto MakeARAStarZero(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<ARAStarZero>(space, heuristic);

    double epsilon;
    params.param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    params.param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (params.getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (params.getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (params.getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (params.getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (params.getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (params.getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    return std::move(search);
}

/// @}
PlannerInterface::PlannerInterface(
    RobotModel* robot,
    CollisionChecker* checker,
    OccupancyGrid* grid)
:
    m_robot(robot),
    m_checker(checker),
    m_grid(grid),
    m_fk_iface(nullptr),
    m_params(),
    m_initialized(false),
    m_pspace(),
    m_heuristics(),
    m_planner(),
    m_sol_cost(INFINITECOST),
    m_planner_id(),
    m_req(),
    m_res()
{
    if (m_robot) {
        m_fk_iface = m_robot->getExtension<ForwardKinematicsInterface>();
    }

    ////////////////////////////////////
    // Setup Planning Space Factories //
    ////////////////////////////////////

    m_space_factories["manip"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeManipLattice(r, c, p, m_grid);
    };

    m_space_factories["manip_lattice_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeManipLatticeEGraph(r, c, p, m_grid);
    };

    m_space_factories["workspace"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeWorkspaceLattice(r, c, p, m_grid);
    };

    m_space_factories["workspace_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeWorkspaceLatticeEGraph(r, c, p, m_grid);
    };

    m_space_factories["adaptive_workspace_lattice"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeAdaptiveWorkspaceLattice(r, c, p, m_grid);
    };

    ///////////////////////////////
    // Setup Heuristic Factories //
    ///////////////////////////////

    m_heuristic_factories["mfbfs"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeMultiFrameBFSHeuristic(space, p, m_grid);
    };

    m_heuristic_factories["bfs"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeBFSHeuristic(space, p, m_grid);
    };

    m_heuristic_factories["euclid"] = MakeEuclidDistHeuristic;

    m_heuristic_factories["joint_distance"] = MakeJointDistHeuristic;

    m_heuristic_factories["bfs_egraph"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeDijkstraEgraphHeuristic3D(space, p, m_grid);
    };

    m_heuristic_factories["joint_distance_egraph"] = MakeJointDistEGraphHeuristic;

    /////////////////////////////
    // Setup Planner Factories //
    /////////////////////////////

    m_planner_factories["arastar"] = MakeARAStar;
    m_planner_factories["awastar"] = MakeAWAStar;
    m_planner_factories["mhastar"] = MakeMHAStar;
    m_planner_factories["larastar"] = MakeLARAStar;
    m_planner_factories["egwastar"] = MakeEGWAStar;
    m_planner_factories["padastar"] = MakePADAStar;

    ////////////////////////////
    // Setup Custom Factories //
    ////////////////////////////

    m_space_factories["workspace_zero"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeWorkspaceLatticeZero(r, c, p, m_grid);
    };

    m_heuristic_factories["workspace_distance"] = MakeWorkspaceDistHeuristic;
    m_planner_factories["arastar_zero"] = MakeARAStarZero;
}

bool PlannerInterface::init(const PlanningParams& params)
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize planner interface");

    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Path: %s", params.shortcut_path ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Type: %s", to_string(params.shortcut_type).c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Interpolate Path: %s", params.interpolate_path ? "true" : "false");

    if (!checkConstructionArgs()) {
        return false;
    }

    if (!checkParams(params)) {
        return false;
    }

    m_params = params;

    m_initialized = true;

    ROS_INFO_NAMED(PI_LOGGER, "Initialized planner interface");
    return m_initialized;
}

bool PlannerInterface::checkConstructionArgs() const
{
    if (!m_robot) {
        ROS_ERROR("Robot Model given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_checker) {
        ROS_ERROR("Collision Checker given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_grid) {
        ROS_ERROR("Occupancy Grid given to Arm Planner Interface must be non-null");
        return false;
    }

    return true;
}

bool PlannerInterface::solve(
    // TODO: this planning scene is probably not being used in any meaningful way
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    clearMotionPlanResponse(req, res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    m_req = req; // record the last attempted request

    if (req.goal_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // TODO: lazily reinitialize planner when algorithm changes
    if (!reinitPlanner(req.planner_id)) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    // plan
    res.trajectory_start = planning_scene.robot_state;
    ROS_INFO_NAMED(PI_LOGGER, "Allowed Time (s): %0.3f", req.allowed_planning_time);

    auto then = clock::now();

    std::vector<RobotState> path;
    if (req.goal_constraints.front().position_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to position!");
        if (!planToPose(req, path, res)) {
            auto now = clock::now();
            res.planning_time = to_seconds(now - then);
            return false;
        }
    } else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to joint configuration!");
        if (!planToConfiguration(req, path, res)) {
            auto now = clock::now();
            res.planning_time = to_seconds(now - then);
            return false;
        }
    } else {
        ROS_ERROR("Both position and joint constraints empty!");
        auto now = clock::now();
        res.planning_time = to_seconds(now - then);
        return false;
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "planner path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        const auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    postProcessPath(path);
    SV_SHOW_INFO_NAMED("trajectory", makePathVisualization(path));

    ROS_DEBUG_NAMED(PI_LOGGER, "smoothed path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        const auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    convertJointVariablePathToJointTrajectory(
            path,
            req.start_state.joint_state.header.frame_id,
            req.start_state.multi_dof_joint_state.header.frame_id,
            res.trajectory);
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    if (!m_params.plan_output_dir.empty()) {
        writePath(res.trajectory_start, res.trajectory);
    }

    profilePath(res.trajectory.joint_trajectory);
//    removeZeroDurationSegments(traj);

    auto now = clock::now();
    res.planning_time = to_seconds(now - then);
    m_res = res; // record the last result
    return true;
}

bool PlannerInterface::solveZero(
    // TODO: this planning scene is probably not being used in any meaningful way
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res,
    bool query,
    bool random_query)
{
    ROS_INFO("Solve Zero");

    clearMotionPlanResponse(req, res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    m_req = req; // record the last attempted request

    if (req.goal_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // make spaces
    auto psait = m_space_factories.find("manip");
    auto manip_space = psait->second(m_robot, m_checker, m_params);
    if (!manip_space) {
        ROS_ERROR("Failed to build manip space");
        return false;
    }

    psait = m_space_factories.find("workspace_zero");
    auto task_space = psait->second(m_robot, m_checker, m_params);
    if (!task_space) {
        ROS_ERROR("Failed to build workspace");
        return false;
    }

    // make heuristics
    auto hait = m_heuristic_factories.find("bfs");
    auto bfs_heuristic = hait->second(manip_space.get(), m_params);
    if (!bfs_heuristic) {
        ROS_ERROR("Failed to build bfs heuristic");
        return false;
    }

    hait = m_heuristic_factories.find("workspace_distance");
    auto wd_heuristic = hait->second(task_space.get(), m_params);
    if (!wd_heuristic) {
        ROS_ERROR("Failed to build wd heuristic");
        return false;
    }

    //insert jd heuristic in the robot planning space
    if (!task_space->insertHeuristic(wd_heuristic.get())) {
        ROS_ERROR("Could not insert heuristic");
        return false;
    }

    //insert bfs heuristic in the robot planning space
    if (!manip_space->insertHeuristic(bfs_heuristic.get())) {
        ROS_ERROR("Could not insert heuristic");
        return false;
    }
    
    // make planners
    auto pait = m_planner_factories.find("arastar");
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name arastar");
        return false;
    }
    auto planner1 = pait->second(manip_space.get(), bfs_heuristic.get(), m_params);

    pait = m_planner_factories.find("arastar_zero");
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name arastar");
        return false;
    }
    auto planner2 = pait->second(task_space.get(), wd_heuristic.get(), m_params);

    // fill start and goal
    RobotState initial_positions;
    if (!fillStartState(m_req.start_state, initial_positions)) {
        ROS_ERROR("Failed to fill start state");
        return false;
    }

    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());
    const auto& goal_constraints = goal_constraints_v.front();

    GoalConstraintZTP goal;
    if (req.goal_constraints.front().position_constraints.size() > 0) {
        if (!fillGoalPositionConstraint(goal_constraints, goal)) {
            ROS_ERROR("Failed to fill goal position constraint");
            return false;
        }
    }
    else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        if (!fillGoalConfigurationConstraint(goal_constraints, goal)) {
            ROS_ERROR("Failed to fill goal configuration constraint");
            return false;
        }
    } else {
        ROS_ERROR("Both position and joint constraints empty!");
    }

    ROS_INFO_NAMED(PI_LOGGER, "Initialized zero time planner");
    m_zero_planner.reset(new ZeroTimePlanner(
        dynamic_cast<ManipLattice*>(manip_space.get()),
        dynamic_cast<WorkspaceLatticeZero*>(task_space.get()),
        dynamic_cast<ARAStar*>(planner1.get()),
        dynamic_cast<ARAStarZero*>(planner2.get())));

    m_zero_planner->getLimits();

    m_zero_planner->setStartAndGoal(initial_positions, goal);

    ROS_INFO("Set start and goal");
    auto task_space_ = dynamic_cast<WorkspaceLatticeZero*>(task_space.get());

    std::vector<RobotState> path;
    if (!query) {
        ROS_INFO("Preprocessing Start Region");
        m_zero_planner->PreProcess(initial_positions);
    }
    else if (!random_query) {
        goal.ws_state.resize(6);
        goal.ws_state[0] = goal.pose.translation()[0];
        goal.ws_state[1] = goal.pose.translation()[1];
        goal.ws_state[2] = goal.pose.translation()[2];
//        while (!task_space_->SampleRobotStateOrientation(goal.angles, goal.ws_state));
        m_zero_planner->setStartAndGoal(initial_positions, goal);
        auto now = clock::now();
        m_zero_planner->GraspQuery(path);
        ROS_INFO_STREAM("Goal ws state: \n" << "Translation:" << goal.ws_state[0] << " " << goal.ws_state[1] << " " << goal.ws_state[2]);
        ROS_INFO_STREAM("Rotation:" << goal.ws_state[3] << " " << goal.ws_state[4] << " " << goal.ws_state[5]);
    }
    else {
        int num_queries = 1;
        double total_time = 0.0;
        double best_time = 10000.0;
        double worst_time = 0.0;
        ROS_INFO("Going to run %d random queries", num_queries);

        for (int i = 0; i < num_queries; ++i) {
            ROS_INFO("\n************* QUERY %d ***************", i);
            ROS_INFO("Zero time query");

            while (!task_space_->SampleRobotState(goal.angles, goal.ws_state));
            m_zero_planner->setStartAndGoal(initial_positions, goal);

//            ROS_INFO_STREAM("Euler before: " << goal.ws_state[3] << " " << goal.ws_state[4] << " " << goal.ws_state[5]);
//            get_euler_zyx(goal.pose.rotation(), goal.ws_state[5], goal.ws_state[4], goal.ws_state[3]);

            ROS_INFO_STREAM("Goal ws state: \n" << "Translation:" << goal.ws_state[0] << " " << goal.ws_state[1] << " " << goal.ws_state[2]);
            ROS_INFO_STREAM("Rotation:" << goal.ws_state[3] << " " << goal.ws_state[4] << " " << goal.ws_state[5]);

            auto now = clock::now();
            m_zero_planner->Query(path);
            res.planning_time = to_seconds(clock::now() - now);

            if (res.planning_time < best_time)
                best_time = res.planning_time;
            if (res.planning_time > worst_time)
                worst_time = res.planning_time;

            total_time += res.planning_time;
            ROS_INFO("ZTP query time: %f", res.planning_time);
            // postProcessPath(path);
            SV_SHOW_INFO_NAMED("trajectory", makePathVisualization(path));

            ROS_DEBUG_NAMED(PI_LOGGER, "smoothed path:");
            for (size_t pidx = 0; pidx < path.size(); ++pidx) {
                const auto& point = path[pidx];
                ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
            }

            convertJointVariablePathToJointTrajectory(
                    path,
                    req.start_state.joint_state.header.frame_id,
                    req.start_state.multi_dof_joint_state.header.frame_id,
                    res.trajectory);
            res.trajectory.joint_trajectory.header.seq = 0;
            res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

            if (!m_params.plan_output_dir.empty()) {
                writePath(res.trajectory_start, res.trajectory);
            }

            profilePath(res.trajectory.joint_trajectory);
        //    removeZeroDurationSegments(traj);

            m_res = res; // record the last result
            // getchar();
        }
        double mean_time = total_time/num_queries;
        ROS_INFO("Mean planning time: %f", mean_time);
        ROS_INFO("Worst planning time: %f", worst_time);
        ROS_INFO("Best planning time: %f", best_time);
    }
    bfs_heuristic.release();    //avoid crash

    return true;
}

bool PlannerInterface::checkParams(
    const PlanningParams& params) const
{
    // TODO: check for existence of planning joints in robot model

    if (params.cost_per_cell < 0) {
        return false;
    }

    return true;
}

bool PlannerInterface::setStart(const moveit_msgs::RobotState& state)
{
    ROS_INFO_NAMED(PI_LOGGER, "set start configuration");

    RobotState initial_positions;

    if (!fillStartState(m_req.start_state, initial_positions)) {
        ROS_ERROR("Failed to fill start state");
        return false;
    }

    ROS_INFO_STREAM_NAMED(PI_LOGGER, "  joint variables: " << initial_positions);

    if (!m_pspace->setStart(initial_positions)) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    for (auto& h : m_heuristics) {
        h.second->updateStart(initial_positions);
    }

    const int start_id = m_pspace->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    if (m_planner->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalConfiguration(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Set goal configuration");

    GoalConstraint goal;
    if (!fillGoalConfigurationConstraint(goal_constraints, goal)) {
        ROS_ERROR("Failed to fill goal configuration constraint");
        return false;
    }

    // TODO: really need to reevaluate the necessity of the planning link
    if (m_fk_iface) {
        goal.pose = m_fk_iface->computeFK(goal.angles);
    } else {
        goal.pose = Eigen::Isometry3d::Identity();
    }

    // set sbpl environment goal
    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    for (auto& h : m_heuristics) {
        h.second->updateGoal(goal);
    }

    // set planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalPosition(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Setting goal position");

    GoalConstraint goal;
    if (!fillGoalPositionConstraint(goal_constraints, goal)) {
        ROS_ERROR("Failed to fill goal position constraint");
        return false;
    }

    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    // set sbpl planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

bool PlannerInterface::plan(double allowed_time, std::vector<RobotState>& path)
{
    // NOTE: this should be done after setting the start/goal in the environment
    // to allow the heuristic to tailor the visualization to the current
    // scenario
    SV_SHOW_DEBUG_NAMED("bfs_walls", getBfsWallsVisualization());
    SV_SHOW_DEBUG_NAMED("bfs_values", getBfsValuesVisualization());

    ROS_WARN_NAMED(PI_LOGGER, "Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    b_ret = m_planner->replan(allowed_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();
        if (!m_pspace->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    return b_ret;
}

bool PlannerInterface::planToPose(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // transform goal pose into reference_frame

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    if (!setGoalPosition(goal_constraints)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(req.allowed_planning_time, path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

bool PlannerInterface::planToConfiguration(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    if (!setGoalConfiguration(goal_constraints)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(req.allowed_planning_time, path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

/// Test if a particular set of goal constraints it supported.
///
/// This tests whether, in general, any planning algorithm supported by this
/// interface can support a particular set of constraints. Certain planning
/// algorithms may not be able to handle a given set of constraints. This
/// method also cannot check for validity of constraints against a particular
/// robot model at this step. In particular, it cannot assert that the a set
/// of joint constraints lists a constraint for each joint, which is currently
/// required.
bool PlannerInterface::SupportsGoalConstraints(
    const std::vector<moveit_msgs::Constraints>& constraints,
    std::string& why)
{
    if (constraints.empty()) {
        return true;
    }

    if (constraints.size() > 1) {
        why = "no planner currently supports more than one goal constraint";
        return false;
    }

    const moveit_msgs::Constraints& constraint = constraints.front();

    if (!constraint.visibility_constraints.empty()) {
        why = "no planner currently supports goal visibility constraints";
        return false;
    }

    // technically multiple goal position/orientation constraints can be
    // solved for if there is one position/orientation volume that entirely
    // bounds all other position/orientation volumes...ignoring for now

    if (constraint.position_constraints.size() > 1) {
        why = "no planner currently supports more than one position constraint";
        return false;
    }

    if (constraint.orientation_constraints.size() > 1) {
        why = "no planner currently supports more than one orientation constraint";
        return false;
    }

    const bool no_pose_constraint =
            constraint.position_constraints.empty() &&
            constraint.orientation_constraints.empty();
    const bool has_pose_constraint =
            constraint.position_constraints.size() == 1 &&
            constraint.orientation_constraints.size() == 1;
    const bool has_joint_constraints = !constraint.joint_constraints.empty();

    if (has_joint_constraints) {
        if (has_pose_constraint) {
            why = "no planner currently supports both pose and joint constraints";
            return false;
        }
    } else {
        if (no_pose_constraint) {
            // no constraints -> ok!
            return true;
        } else if (has_pose_constraint) {
            if (constraint.position_constraints.front().link_name !=
                constraint.orientation_constraints.front().link_name)
            {
                why = "pose constraint must be for a single link";
                return false;
            }
            return true;
        } else {
            // pose constraint is invalid
            why = "no planner supports only one position constraint or one orientation constraint";
            return false;
        }
    }

    // made it through the gauntlet
    return true;
}

bool PlannerInterface::canServiceRequest(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    // check for an empty start state
    // TODO: generalize this to "missing necessary state information"
    if (req.start_state.joint_state.position.empty()) {
        ROS_ERROR("No start state given. Unable to plan.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    // check if position & orientation constraints is empty
    const moveit_msgs::Constraints& goal_constraints =
            req.goal_constraints.front();

    if ((
            goal_constraints.position_constraints.empty() ||
            goal_constraints.orientation_constraints.empty()
        ) &&
        goal_constraints.joint_constraints.empty())
    {
        ROS_ERROR("Position or orientation constraint is empty");
        ROS_ERROR("Joint constraint is empty");
        ROS_ERROR("PlannerInterface expects a 6D pose constraint or set of joint constraints");
        return false;
    }

    // check if there is more than one goal constraint
    if (goal_constraints.position_constraints.size() > 1 ||
        goal_constraints.orientation_constraints.size() > 1)
    {
        ROS_WARN_NAMED(PI_LOGGER, "The planning request message contains %zd position and %zd orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", goal_constraints.position_constraints.size(), goal_constraints.orientation_constraints.size());
    }

    return true;
}

std::map<std::string, double> PlannerInterface::getPlannerStats()
{
    std::map<std::string, double> stats;
    stats["initial solution planning time"] = m_planner->get_initial_eps_planning_time();
    stats["initial epsilon"] = m_planner->get_initial_eps();
    stats["initial solution expansions"] = m_planner->get_n_expands_init_solution();
    stats["final epsilon planning time"] = m_planner->get_final_eps_planning_time();
    stats["final epsilon"] = m_planner->get_final_epsilon();
    stats["solution epsilon"] = m_planner->get_solution_eps();
    stats["expansions"] = m_planner->get_n_expands();
    stats["solution cost"] = m_sol_cost;
    return stats;
}

auto PlannerInterface::makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }

    double cinc = 1.0 / double(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_checker->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            const float r = 0.1f;
            const float g = cinc * (float)(path.size() - (i + 1));
            const float b = cinc * (float)i;
            marker.color = visual::Color{ r, g, b, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory";
        marker.id = i;
    }

    return ma;
}

auto PlannerInterface::getBfsValuesVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getValuesVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getValuesVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getValuesVisualization();
    } else {
        return visual::Marker{ };
    }
}

auto PlannerInterface::getBfsWallsVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getWallsVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getWallsVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getWallsVisualization();
    } else {
        return visual::Marker{ };
    }
}

bool PlannerInterface::fillStartState(
    const moveit_msgs::RobotState& state,
    RobotState& start_state)
{
    if (!state.multi_dof_joint_state.joint_names.empty()) {
        const auto& mdof_joint_names = state.multi_dof_joint_state.joint_names;
        for (const std::string& joint_name : m_robot->getPlanningJoints()) {
            auto it = std::find(mdof_joint_names.begin(), mdof_joint_names.end(), joint_name);
            if (it != mdof_joint_names.end()) {
                ROS_WARN_NAMED(PI_LOGGER, "planner does not currently support planning for multi-dof joints. found '%s' in planning joints", joint_name.c_str());
            }
        }
    }

    // RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state,
            state.multi_dof_joint_state,
            m_robot->getPlanningJoints(),
            start_state,
            missing))
    {
        ROS_ERROR("start state is missing planning joints: ");
//        SMPL_WARN_STREAM("start state is missing planning joints: " << missing);
        return false;
    }
    return true;
}

bool PlannerInterface::fillGoalConfigurationConstraint(
    const moveit_msgs::Constraints& goal_constraints,
    GoalConstraint& goal)
{
    std::vector<double> sbpl_angle_goal(m_robot->jointVariableCount(), 0);
    std::vector<double> sbpl_angle_tolerance(m_robot->jointVariableCount(), angles::to_radians(3.0));

    if (goal_constraints.joint_constraints.size() < m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "All %zu arm joint constraints must be specified for goal!", m_robot->jointVariableCount());
        return false;
    }
    if (goal_constraints.joint_constraints.size() > m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "%d joint constraints specified! Using the first %zu!", (int)goal_constraints.joint_constraints.size(), m_robot->jointVariableCount());
        return false;
    }

    const size_t num_angle_constraints = std::min(
            goal_constraints.joint_constraints.size(), sbpl_angle_goal.size());
    for (size_t i = 0; i < num_angle_constraints; i++) {
        const auto& joint_constraint = goal_constraints.joint_constraints[i];
        const std::string& joint_name = joint_constraint.joint_name;
        auto jit = std::find(
                m_robot->getPlanningJoints().begin(),
                m_robot->getPlanningJoints().end(),
                joint_name);
        if (jit == m_robot->getPlanningJoints().end()) {
            ROS_ERROR("Failed to find goal constraint for joint '%s'", joint_name.c_str());
            return false;
        }
        int jidx = std::distance(m_robot->getPlanningJoints().begin(), jit);
        sbpl_angle_goal[jidx] = joint_constraint.position;
        sbpl_angle_tolerance[jidx] = std::min(
                fabs(joint_constraint.tolerance_above),
                fabs(joint_constraint.tolerance_below));
        ROS_INFO_NAMED(PI_LOGGER, "Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, joint_name.c_str(), sbpl_angle_goal[jidx], sbpl_angle_tolerance[jidx]);
    }

    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = sbpl_angle_goal;
    goal.angle_tolerances = sbpl_angle_tolerance;
}

bool PlannerInterface::fillGoalPositionConstraint(
    const moveit_msgs::Constraints& goal_constraints,
    GoalConstraint& goal)
{
    Eigen::Isometry3d goal_pose;
    Eigen::Vector3d offset;
    if (!extractGoalPoseFromGoalConstraints(
            goal_constraints, goal_pose, offset))
    {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal pose from goal constraints");
        return false;
    }

    goal.type = GoalType::XYZ_RPY_GOAL;
    goal.pose = goal_pose;

    std::vector<double> sbpl_tolerance(6, 0.0);
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0])) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    goal.xyz_tolerance[0] = sbpl_tolerance[0];
    goal.xyz_tolerance[1] = sbpl_tolerance[1];
    goal.xyz_tolerance[2] = sbpl_tolerance[2];
    goal.rpy_tolerance[0] = sbpl_tolerance[3];
    goal.rpy_tolerance[1] = sbpl_tolerance[4];
    goal.rpy_tolerance[2] = sbpl_tolerance[5];

    ROS_INFO_NAMED(PI_LOGGER, "New Goal");
    double yaw, pitch, roll;
    angles::get_euler_zyx(goal.pose.rotation(), yaw, pitch, roll);
    ROS_INFO_NAMED(PI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f)", goal.pose.translation()[0], goal.pose.translation()[1], goal.pose.translation()[2], roll, pitch, yaw);
    ROS_INFO_NAMED(PI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)", sbpl_tolerance[0], sbpl_tolerance[1], sbpl_tolerance[2], sbpl_tolerance[3], sbpl_tolerance[4], sbpl_tolerance[5]);
    return true;
}

bool PlannerInterface::extractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Isometry3d& goal_pose,
    Eigen::Vector3d& offset) const
{
    if (constraints.position_constraints.empty() ||
        constraints.orientation_constraints.empty())
    {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one position and one orientation constraint");
        return false;
    }

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?
    const moveit_msgs::PositionConstraint& position_constraint = constraints.position_constraints.front();
    const moveit_msgs::OrientationConstraint& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    const shape_msgs::SolidPrimitive& bounding_primitive = position_constraint.constraint_region.primitives.front();
    const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Isometry3d T_planning_eef = // T_planning_off * T_off_eef;
            Eigen::Translation3d(
                    primitive_pose.position.x,
                    primitive_pose.position.y,
                    primitive_pose.position.z) *
            Eigen::Quaterniond(
                    primitive_pose.orientation.w,
                    primitive_pose.orientation.x,
                    primitive_pose.orientation.y,
                    primitive_pose.orientation.z);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());

    Eigen::Quaterniond eef_orientation;
    tf::quaternionMsgToEigen(orientation_constraint.orientation, eef_orientation);

    goal_pose = Eigen::Translation3d(eef_pos) * eef_orientation;

    tf::vectorMsgToEigen(position_constraint.target_point_offset, offset);
    return true;
}

bool PlannerInterface::extractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double* tol)
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front()
                .constraint_region.primitives.empty())
    {
        const moveit_msgs::PositionConstraint& position_constraint =
                goal_constraints.position_constraints.front();
        const shape_msgs::SolidPrimitive& constraint_primitive =
                position_constraint.constraint_region.primitives.front();
        const std::vector<double>& dims = constraint_primitive.dimensions;
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tol[0] = dims[shape_msgs::SolidPrimitive::BOX_X];
            tol[1] = dims[shape_msgs::SolidPrimitive::BOX_Y];
            tol[2] = dims[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tol[0] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tol[0] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tol[0] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    }
    else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        const std::vector<moveit_msgs::OrientationConstraint>& orientation_constraints = goal_constraints.orientation_constraints;
        const moveit_msgs::OrientationConstraint& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    }
    else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

void PlannerInterface::clearMotionPlanResponse(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
//    res.trajectory_start.joint_state;
//    res.trajectory_start.multi_dof_joint_state;
//    res.trajectory_start.attached_collision_objects;
    res.trajectory_start.is_diff = false;
    res.group_name = req.group_name;
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.joint_trajectory.header.frame_id = "";
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.multi_dof_joint_trajectory.header.seq = 0;
    res.trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.multi_dof_joint_trajectory.header.frame_id = "";
    res.trajectory.multi_dof_joint_trajectory.joint_names.clear();
    res.trajectory.multi_dof_joint_trajectory.points.clear();
    res.planning_time = 0.0;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
}

bool PlannerInterface::parsePlannerID(
    const std::string& planner_id,
    std::string& space_name,
    std::string& heuristic_name,
    std::string& search_name) const
{
    boost::regex alg_regex("(\\w+)(?:\\.(\\w+))?(?:\\.(\\w+))?");

    boost::smatch sm;

    ROS_INFO("Match planner id '%s' against regex '%s'", planner_id.c_str(), alg_regex.str().c_str());
    if (!boost::regex_match(planner_id, sm, alg_regex)) {
        return false;
    }

    const std::string default_search_name = "arastar";
    const std::string default_heuristic_name = "bfs";
    const std::string default_space_name = "manip";

    if (sm.size() < 2 || sm[1].str().empty()) {
        search_name = default_search_name;
    } else {
        search_name = sm[1];
    }

    if (sm.size() < 3 || sm[2].str().empty()) {
        heuristic_name = default_heuristic_name;
    } else {
        heuristic_name = sm[2];
    }

    if (sm.size() < 4 || sm[3].str().empty()) {
        space_name = default_space_name;
    } else {
        space_name = sm[3];
    }

    return true;
}

void PlannerInterface::clearGraphStateToPlannerStateMap()
{
    if (!m_pspace) {
        return;
    }

    std::vector<int*>& state_id_to_index = m_pspace->StateID2IndexMapping;
    for (int* mapping : state_id_to_index) {
        for (int i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
            mapping[i] = -1;
        }
    }
}

bool PlannerInterface::reinitPlanner(const std::string& planner_id)
{
    if (planner_id == m_planner_id) {
        // TODO: check for specification of default planning components when
        // they may not have been previously specified
        return true;
    }

    ROS_INFO_NAMED(PI_LOGGER, "Initialize planner");

    std::string search_name;
    std::string heuristic_name;
    std::string space_name;
    if (!parsePlannerID(planner_id, space_name, heuristic_name, search_name)) {
        ROS_ERROR("Failed to parse planner setup");
        return false;
    }

    ROS_INFO_NAMED(PI_LOGGER, " -> Planning Space: %s", space_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Heuristic: %s", heuristic_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Search: %s", search_name.c_str());

    auto psait = m_space_factories.find(space_name);
    if (psait == m_space_factories.end()) {
        ROS_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    m_pspace = psait->second(m_robot, m_checker, m_params);
    if (!m_pspace) {
        ROS_ERROR("Failed to build planning space '%s'", space_name.c_str());
        return false;
    }

    auto hait = m_heuristic_factories.find(heuristic_name);
    if (hait == m_heuristic_factories.end()) {
        ROS_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    auto heuristic = hait->second(m_pspace.get(), m_params);
    if (!heuristic) {
        ROS_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics.clear();
    m_heuristics.insert(std::make_pair(heuristic_name, std::move(heuristic)));

    for (const auto& entry : m_heuristics) {
        m_pspace->insertHeuristic(entry.second.get());
    }

    auto pait = m_planner_factories.find(search_name);
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    auto first_heuristic = begin(m_heuristics);
    m_planner = pait->second(m_pspace.get(), first_heuristic->second.get(), m_params);
    if (!m_planner) {
        ROS_ERROR("Failed to build planner '%s'", search_name.c_str());
        return false;
    }
    m_planner_id = planner_id;
    return true;
}

void PlannerInterface::profilePath(trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    auto& joint_names = traj.joint_names;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev_point = traj.points[i - 1];
        auto& curr_point = traj.points[i];

        // find the maximum time required for any joint to reach the next
        // waypoint
        double max_time = 0.0;
        for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
            const double from_pos = prev_point.positions[jidx];
            const double to_pos = curr_point.positions[jidx];
            const double vel = m_robot->velLimit(jidx);
            if (vel <= 0.0) {
                continue;
            }
            double t = 0.0;
            if (m_robot->isContinuous(jidx)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }

            max_time = std::max(max_time, t);
        }

        curr_point.time_from_start = prev_point.time_from_start + ros::Duration(max_time);
    }
}

void PlannerInterface::removeZeroDurationSegments(
    trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    // filter out any duplicate points
    // TODO: find out where these are happening
    size_t end_idx = 1; // current end of the non-filtered range
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev = traj.points[end_idx - 1];
        auto& curr = traj.points[i];
        if (curr.time_from_start != prev.time_from_start) {
            ROS_INFO("Move index %zu into %zu", i, end_idx);
            if (end_idx != i) {
                traj.points[end_idx] = std::move(curr);
            }
            end_idx++;
        }
    }
    traj.points.resize(end_idx);
}

bool PlannerInterface::isPathValid(const std::vector<RobotState>& path) const
{
    for (size_t i = 1; i < path.size(); ++i) {
        if (!m_checker->isStateToStateValid(path[i - 1], path[i])) {
            ROS_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
            return false;
        }
    }
    return true;
}

void PlannerInterface::postProcessPath(std::vector<RobotState>& path) const
{
    const bool check_planned_path = true;
    if (check_planned_path && !isPathValid(path)) {
        ROS_ERROR("Planned path is invalid");
    }

    // shortcut path
    if (m_params.shortcut_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate planned path with %zu waypoints before shortcutting.", path.size());
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        } else {
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        }
    }

    // interpolate path
    if (m_params.interpolate_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate trajectory");
        }
    }
}

void PlannerInterface::convertJointVariablePathToJointTrajectory(
    const std::vector<RobotState>& path,
    const std::string& joint_state_frame,
    const std::string& multi_dof_joint_state_frame,
    moveit_msgs::RobotTrajectory& traj) const
{
    ROS_INFO("Convert Variable Path to Robot Trajectory");

    traj.joint_trajectory.header.frame_id = joint_state_frame;
    traj.multi_dof_joint_trajectory.header.frame_id = multi_dof_joint_state_frame;

    traj.joint_trajectory.joint_names.clear();
    traj.joint_trajectory.points.clear();
    traj.multi_dof_joint_trajectory.joint_names.clear();
    traj.multi_dof_joint_trajectory.points.clear();

    // fill joint names header for both single- and multi-dof joint trajectories
    auto& variable_names = m_robot->getPlanningJoints();
    for (auto& var_name : variable_names) {
        std::string joint_name;
        if (IsMultiDOFJointVariable(var_name, &joint_name)) {
            auto it = std::find(
                    begin(traj.multi_dof_joint_trajectory.joint_names),
                    end(traj.multi_dof_joint_trajectory.joint_names),
                    joint_name);
            if (it == end(traj.multi_dof_joint_trajectory.joint_names)) {
                // avoid duplicates
                traj.multi_dof_joint_trajectory.joint_names.push_back(joint_name);
            }
        } else {
            traj.joint_trajectory.joint_names.push_back(var_name);
        }
    }

    ROS_INFO("  Path includes %zu single-dof joints and %zu multi-dof joints",
            traj.joint_trajectory.joint_names.size(),
            traj.multi_dof_joint_trajectory.joint_names.size());

    // empty or number of points in the path
    if (!traj.joint_trajectory.joint_names.empty()) {
        traj.joint_trajectory.points.resize(path.size());
    }
    // empty or number of points in the path
    if (!traj.multi_dof_joint_trajectory.joint_names.empty()) {
        traj.multi_dof_joint_trajectory.points.resize(path.size());
    }

    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];

        for (size_t vidx = 0; vidx < point.size(); ++vidx) {
            auto& var_name = variable_names[vidx];

            std::string joint_name, local_name;
            if (IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto& p = traj.multi_dof_joint_trajectory.points[pidx];
                p.transforms.resize(traj.multi_dof_joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.multi_dof_joint_trajectory.joint_names),
                        end(traj.multi_dof_joint_trajectory.joint_names),
                        joint_name);
                if (it == end(traj.multi_dof_joint_trajectory.joint_names)) continue;

                auto tidx = std::distance(begin(traj.multi_dof_joint_trajectory.joint_names), it);

                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    p.transforms[tidx].translation.x = point[vidx];
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    p.transforms[tidx].translation.y = point[vidx];
                } else if (local_name == "trans_z") {
                    p.transforms[tidx].translation.z = point[vidx];
                } else if (local_name == "theta") {
                    Eigen::Quaterniond q(Eigen::AngleAxisd(point[vidx], Eigen::Vector3d::UnitZ()));
                    tf::quaternionEigenToMsg(q, p.transforms[tidx].rotation);
                } else if (local_name == "rot_w") {
                    p.transforms[tidx].rotation.w = point[vidx];
                } else if (local_name == "rot_x") {
                    p.transforms[tidx].rotation.x = point[vidx];
                } else if (local_name == "rot_y") {
                    p.transforms[tidx].rotation.y = point[vidx];
                } else if (local_name == "rot_z") {
                    p.transforms[tidx].rotation.z = point[vidx];
                } else {
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }
            } else {
                auto& p = traj.joint_trajectory.points[pidx];
                p.positions.resize(traj.joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.joint_trajectory.joint_names),
                        end(traj.joint_trajectory.joint_names),
                        var_name);
                if (it == end(traj.joint_trajectory.joint_names)) continue;

                auto posidx = std::distance(begin(traj.joint_trajectory.joint_names), it);

                p.positions[posidx] = point[vidx];
            }
        }
    }
}

bool PlannerInterface::writePath(
    const moveit_msgs::RobotState& ref,
    const moveit_msgs::RobotTrajectory& traj) const
{
    boost::filesystem::path p(m_params.plan_output_dir);

    try {
        if (!boost::filesystem::exists(p)) {
            ROS_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            ROS_ERROR("Failed to log path. %s is not a directory", m_params.plan_output_dir.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        ROS_ERROR("Failed to create plan output directory %s", p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = clock::now();
    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    ROS_INFO("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
        const std::string& var_name = m_robot->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != m_robot->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << '\n';

    const size_t wp_count = std::max(
            traj.joint_trajectory.points.size(),
            traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            const trajectory_msgs::JointTrajectoryPoint& wp =
                    traj.joint_trajectory.points[widx];
            const size_t joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string& joint_name =
                        traj.joint_trajectory.joint_names[jidx];
                double vp = wp.positions[jidx];
                auto it = std::find(
                        state.joint_state.name.begin(),
                        state.joint_state.name.end(),
                        joint_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx = std::distance(state.joint_state.name.begin(), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint& wp =
                    traj.multi_dof_joint_trajectory.points[widx];
            const size_t joint_count = traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string& joint_name =
                        traj.multi_dof_joint_trajectory.joint_names[jidx];
                const geometry_msgs::Transform& t = wp.transforms[jidx];
                auto it = std::find(
                        state.multi_dof_joint_state.joint_names.begin(),
                        state.multi_dof_joint_state.joint_names.end(),
                        joint_name);
                if (it != state.multi_dof_joint_state.joint_names.end()) {
                    size_t tvidx = std::distance(state.multi_dof_joint_state.joint_names.begin(), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
            const std::string& var_name = m_robot->getPlanningJoints()[vidx];
            const bool var_is_mdof = false; // TODO: multi-dof joints in robot model
            if (var_is_mdof) {

            } else {
                auto it = std::find(
                        state.joint_state.name.begin(),
                        state.joint_state.name.end(),
                        var_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx = std::distance(state.joint_state.name.begin(), it);
                    double vp = state.joint_state.position[tvidx];
                    ofs << vp;
                    if (vidx != m_robot->jointVariableCount() - 1) {
                        ofs << ',';
                    }
                }
            }
        }
        ofs << '\n';
    }

    return true;
}

} // namespace smpl
