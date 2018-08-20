////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, Fahad Islam
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

/// \author Fahad Islam

// standard includes
#include <stdio.h>
#include <stdlib.h>

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <tf_conversions/tf_eigen.h>

// project includes
#include <smpl_ztp/ros/zero_time_planner.h>

namespace smpl {

const char* PI_LOGGER_ZERO = "simple_zero";

namespace
{
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

    if (fCfg == NULL) {
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
        object_ids.push_back(sTemp);

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
}

ZeroTimePlanner::ZeroTimePlanner(
	ManipLattice* manip_space,
	WorkspaceLatticeZero* task_space,
	ARAStar* planner,
	ARAStarZero* planner_zero)
:
    m_manip_space(manip_space),
    m_task_space(task_space),
    m_planner(planner),
    m_planner_zero(planner_zero)
{
    if (!m_nh.getParam("preprocess_planner", m_pp_planner)) {
        ROS_ERROR("Could not find preprocessing planner name");
    }
    if (m_regions.empty()) {
        ReadRegions();
        m_task_space->PassRegions(&m_regions, &m_iregions);
    }
}

bool ZeroTimePlanner::isQueryCovered(
    const RobotState& full_start_state,
    const GoalConstraint& goal)
{
    // check if preprocessing covered this query
    if (!m_task_space->IsQueryCovered(full_start_state, goal)) {
        return false;
    }

    return true;
}

void ZeroTimePlanner::setStartAndGoal(
    const RobotState& start_state,
    const GoalConstraint& goal)
{
    m_start_state = start_state;
    m_goal = goal;
}

void ZeroTimePlanner::InitMoveitOMPL()
{
    m_group.reset(new moveit::planning_interface::MoveGroup("right_arm"));
    ROS_INFO("Planning path with OMPL");

    // Collision objects
    auto object_filename = "/usr0/home/fi/smpl_ws/src/smpl_ztp/env/tabletop.env";
    auto objects = GetCollisionObjects(object_filename, m_group->getPlanningFrame());
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    for (auto o : objects) {
        collision_objects.push_back(o);
    }
    ROS_INFO("Add an object into the world");
    m_planning_scene_interface.addCollisionObjects(collision_objects);

    // planner settings
    m_group->setPlanningTime(5.0);
    m_group->setPlannerId(m_pp_planner);
}

bool ZeroTimePlanner::PlanPathFromStartToAttractorOMPL(const RobotState& attractor, std::vector<RobotState>& path)
{
    ROS_INFO("Planning path with OMPL");

    // start -> actual start
    // goal -> attractor

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // set start and goal
    // geometry_msgs::Pose target_pose;
    // tf::poseEigenToMsg(m_goal.pose, target_pose);
    // m_group->setPoseTarget(target_pose);
    robot_state::RobotState goal_state(*m_group->getCurrentState());
    goal_state.setJointGroupPositions("right_arm", attractor);
    m_group->setJointValueTarget(goal_state);

    robot_state::RobotState start_state(*m_group->getCurrentState());
    start_state.setJointGroupPositions("right_arm", m_start_state);
    m_group->setStartState(start_state);

    // plan
    ROS_INFO("Going to plan!");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    auto ret = m_group->plan(my_plan);
    sleep(0.1);

    if (ret != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_WARN("OMPL failed to plan");
        return false;
    }
    else {
        ROS_INFO("Solution found by OMPL");
    }

    // fill path
    path.resize(my_plan.trajectory_.joint_trajectory.points.size());
    for (size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i) {
        auto positions = my_plan.trajectory_.joint_trajectory.points[i].positions;
        std::vector<double> wp;
        unsigned dataArraySize = sizeof(positions) / sizeof(positions[0]);
        std::copy(&positions[0], &positions[dataArraySize], back_inserter(path[i]));
    }
    return true;
}

bool ZeroTimePlanner::PlanPathFromStartToAttractorSMPL(const RobotState& attractor, std::vector<RobotState>& path)
{
    ROS_INFO("Planning path with SMPL");

    // start -> actual start
    // goal -> attractor

    m_manip_space->setStart(m_start_state);
    const int start_id_manip_space = m_manip_space->getStartStateID();
    if (start_id_manip_space == -1) {
        ROS_ERROR("No start state has been set in manip lattice");
        return false;
    }

    if (m_planner->set_start(start_id_manip_space) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    GoalConstraint goal;
    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = attractor;
    goal.angle_tolerances.resize(attractor.size());
    std::fill(goal.angle_tolerances.begin(),goal.angle_tolerances.end(), smpl::angles::to_radians(1.0));
    m_manip_space->setGoal(goal);

    // set sbpl planner goal
    const int goal_id = m_manip_space->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    int m_sol_cost;
    double allowed_planning_time = 10.0;
    b_ret = m_planner->replan(allowed_planning_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER_ZERO, "Path returned by the planner is empty?");
        b_ret = false;
    }

    if (!b_ret) {
        ROS_WARN("Solution not found");
        return false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Solution Cost: %d", m_sol_cost);

        if (!m_manip_space->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    // free manip space
    m_manip_space->clearStates();
    m_planner->force_planning_from_scratch_and_free_memory();
    return true;
}

void ZeroTimePlanner::PreProcess(const RobotState& full_start_state)
{
    m_regions.clear();
    if (m_pp_planner != "ARAStar")
        InitMoveitOMPL();

    unsigned int radius_max = 1000;
    m_task_space->PassRegions(&m_regions, &m_iregions);

    // 1. SAMPLE ATTRACTOR
    int maximum_tries = 10000;
    WorkspaceState sampled_state;
    // also sets the attractor as goal for heuristic functions
    int sampled_state_id = m_task_space->SampleAttractorState(sampled_state, maximum_tries);
    m_task_space->VisualizePoint(sampled_state_id, "attractor");
    if (sampled_state_id == -1) {
        // ROS_INFO("Converged! I tried for %d samples", maximum_tries);
        ROS_ERROR("Failed to sample first attractor");
        return;
    }

    m_valid_front.insert(sampled_state);
    while (!m_valid_front.empty() || !m_invalid_front.empty()) {
    	while (!m_valid_front.empty()) {
            auto it = m_valid_front.begin();
        	WorkspaceState attractor = *it;
            m_valid_front.erase(it);
        	int attractor_state_id = m_task_space->SetAttractorState(attractor);
            if (!m_task_space->IsStateCovered(true, attractor_state_id) &&
                m_bad_attractors.find(attractor) == m_bad_attractors.end()) {
            	m_task_space->VisualizePoint(sampled_state_id, "attractor");
    	        std::vector<RobotState> path;
    #if 1
                // 2. PLAN PATH TO ACTUAL GOAL
                RobotState attractor_joint_state;
                m_task_space->GetJointState(attractor_state_id, attractor_joint_state);

                bool ret;
                if (m_pp_planner != "ARAStar") {
                    ret = PlanPathFromStartToAttractorOMPL(attractor_joint_state, path);
                }
                else {
                    ret = PlanPathFromStartToAttractorSMPL(attractor_joint_state, path);
                }

                if (!ret) {
                    m_bad_attractors.insert(attractor);
                    continue;   //TODO: label that state as unreachable
                }
                // getchar();
    #endif
    	        // 3. COMPUTE REACHABILITY
    	        m_task_space->UpdateSearchMode(REACHABILITY);

    	        // reinitialize the search space
    	        m_planner_zero->force_planning_from_scratch();

    	        // reachability search
    	        int radius = m_planner_zero->compute_reachability(radius_max, attractor_state_id);

    	        // 4. ADD REGION
    	        region r;
                r.start = full_start_state;
    	        r.radius = radius;
    	        r.state = attractor;
    	        r.path = path;
    	        m_regions.push_back(r);

    	        ROS_INFO("Radius %d, Regions so far %zu", radius, m_regions.size());

    	        // coverage
    	        // m_task_space->PruneCoveredStates(m_valid_front);
             //    m_task_space->PruneCoveredStates(m_invalid_front);

    	        std::vector<int> open;
    	        m_planner_zero->get_frontier_stateids(open);

    	        std::vector<WorkspaceState> valid_states;
    	        std::vector<WorkspaceState> invalid_states;
    	        m_task_space->GetUncoveredFrontierStates(open, valid_states, invalid_states);
    	        // m_valid_front.insert(valid_states.begin(), valid_states.end());
    	        // m_invalid_front.insert(invalid_states.begin(), invalid_states.end());
                std::copy( valid_states.begin(), valid_states.end(), std::inserter( m_valid_front, m_valid_front.end() ) );
                std::copy( invalid_states.begin(), invalid_states.end(), std::inserter( m_invalid_front, m_invalid_front.end() ) );

                ROS_INFO("VALID:");
                ROS_INFO("Total frontier: %zu, (valid: %zu, invalid: %zu)",
                    open.size(), valid_states.size(), invalid_states.size());
                ROS_INFO("Overall! valid: %zu invalid: %zu\n", m_valid_front.size(), m_invalid_front.size());
                // getchar();
    	        // free task space
    	        m_task_space->ClearStates();
    	        m_planner_zero->force_planning_from_scratch_and_free_memory();
            }
	    }

        while (!m_invalid_front.empty()) {
            auto it = m_invalid_front.begin();
            WorkspaceState istate = *it;
            m_invalid_front.erase(it);
            int iv_start_state_id = m_task_space->SetInvalidStartState(istate);
            if (!m_task_space->IsStateCovered(true, iv_start_state_id)
                && !m_task_space->IsStateCovered(false, iv_start_state_id)) {
                m_task_space->VisualizePoint(sampled_state_id, "attractor");

                int radius = m_planner_zero->search_for_valid_uncovered_states(radius_max, iv_start_state_id);
                // m_task_space->VisualizePoint(v_state_id, "test");
                region r;
                r.radius = radius;
                r.state = istate;
                m_iregions.push_back(r);

                ROS_INFO("Radius %d, IRegions so far %zu", radius, m_iregions.size());

                // m_task_space->PruneCoveredStates(m_valid_front);
                // m_task_space->PruneCoveredStates(m_invalid_front);

                std::vector<int> open;
                m_planner_zero->get_frontier_stateids(open);
                std::vector<WorkspaceState> valid_states;
                std::vector<WorkspaceState> invalid_states;
                m_task_space->GetUncoveredFrontierStates(open, valid_states, invalid_states);
                // m_valid_front.insert(valid_states.begin(), valid_states.end());
                // m_invalid_front.insert(invalid_states.begin(), invalid_states.end());
                std::copy( valid_states.begin(), valid_states.end(), std::inserter( m_valid_front, m_valid_front.end() ) );
                std::copy( invalid_states.begin(), invalid_states.end(), std::inserter( m_invalid_front, m_invalid_front.end() ) );
                ROS_INFO("INVALID:");
                ROS_INFO("Total frontier: %zu, (valid: %zu, invalid: %zu)",
                    open.size(), valid_states.size(), invalid_states.size());
                ROS_INFO("Overall! valid: %zu invalid: %zu\n", m_valid_front.size(), m_invalid_front.size());
                // getchar();

                if (m_valid_front.size() > 0) {
                    // if (valid_states.size() > 1) {
                    //     ROS_ERROR("Found more than one valid state %zu", valid_states.size());
                    // }
                    break;
                }
            }
            m_task_space->ClearStates();
            m_planner_zero->force_planning_from_scratch_and_free_memory();
        }
    }
    m_task_space->PruneRegions();
    WriteRegions();
}

void ZeroTimePlanner::Query(std::vector<RobotState>& path)
{
    m_task_space->UpdateSearchMode(QUERY);

    // start -> actual goal
    // goal -> attractor

    RobotState start_state;
#if 1
    start_state = m_goal.angles;
#else	// select random start
    while (!m_task_space->SampleRobotState(start_state));
#endif

    if (!m_task_space->IsRobotStateInGoalRegion(start_state)) {
    	ROS_ERROR("Query state outside start region");
    	return;
    }

    int reg_idx = m_task_space->FindRegionContainingState(start_state);

    if (reg_idx == -1) {
        ROS_ERROR("Query start state not covered");
        return;
    }

    // set start
    m_task_space->setStart(start_state);
    const int start_id = m_task_space->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set in workspace lattice");
        return;
    }

    if (m_planner_zero->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return;
    }

    GoalConstraint goal;
    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = m_regions[reg_idx].state;
	m_task_space->setGoal(goal);

    // set sbpl planner goal
    const int goal_id = m_task_space->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return;
    }

    if (m_planner_zero->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return;
    }

    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner_zero->force_planning_from_scratch();

    // plan
    int m_sol_cost;
    b_ret = m_planner_zero->replan(100, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER_ZERO, "Path returned by the planner is empty?");
        b_ret = false;
    }

    if (!b_ret) {
        ROS_ERROR("Planner failed in query phase");
    }

    // if a path is returned, then pack it into msg form
    std::vector<RobotState> ztp_path;
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "Planning succeeded");
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Initial): %d", m_planner_zero->get_n_expands_init_solution());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Final): %d", m_planner_zero->get_n_expands());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Epsilon (Initial): %0.3f", m_planner_zero->get_initial_eps());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Epsilon (Final): %0.3f", m_planner_zero->get_solution_eps());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Time (Initial): %0.3f", m_planner_zero->get_initial_eps_planning_time());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Time (Final): %0.3f", m_planner_zero->get_final_eps_planning_time());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Solution Cost: %d", m_sol_cost);

        if (!m_task_space->extractPath(solution_state_ids, ztp_path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return;
        }

        std::reverse(ztp_path.begin(), ztp_path.end());   // path from to attractor to goal
        path = m_regions[reg_idx].path;
        path.insert(
            path.end(),
            ztp_path.begin(),
            ztp_path.end());

        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "Preprocessed path length: %zu", m_regions[reg_idx].path.size());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "Query path length:        %zu", ztp_path.size());
    }

    m_task_space->ClearStates();
    m_planner_zero->force_planning_from_scratch_and_free_memory();
}

void ZeroTimePlanner::WriteRegions()
{
    // sort
    std::sort(m_regions.begin(), m_regions.end(), [] (const region &a,
              const region &b)
    {
        return (a.radius > b.radius);
    });

	ROS_INFO("Writing regions to file");
    boost::filesystem::path myFile = boost::filesystem::current_path() / "myfile.dat";
    boost::filesystem::ofstream ofs(myFile);
    boost::archive::text_oarchive ta(ofs);
    ta << m_regions;
}

void ZeroTimePlanner::ReadRegions()
{
	ROS_INFO("Reading regions from file");
    try {
        boost::filesystem::path myFile = boost::filesystem::current_path() / "myfile.dat";
        boost::filesystem::ifstream ifs(myFile/*.native()*/);
        boost::archive::text_iarchive ta(ifs);
        ta >> m_regions;
    }
    catch (...) {
        ROS_WARN("Unable to read preprocessed file");
    }
}

ZeroTimePlanner::~ZeroTimePlanner()
{
}

} // namespace smpl
