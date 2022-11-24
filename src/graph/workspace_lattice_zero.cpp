////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush, Fahad Islam
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
/// \author Fahad Islam
#define SMPL_CONSOLE_ROS
#include <smpl_ztp/graph/workspace_lattice_zero.h>

// system includes
#include <chrono>
#include <boost/functional/hash.hpp>
#include <fstream>

// project includes
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/graph/workspace_lattice_action_space.h>

#define tie_breaking

namespace smpl {

WorkspaceLatticeZero::~WorkspaceLatticeZero()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    // NOTE: StateID2IndexMapping cleared by DiscreteSpaceInformation
}

bool WorkspaceLatticeZero::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const Params& _params,
    WorkspaceLatticeActionSpace* actions)
{
    if (!WorkspaceLattice::init(_robot, checker, _params, actions)) {
        return false;
    }

    // this should serve as a reasonable dummy state since no valid state should
    // have an empty coordinate vector

    // Doing it in FindRegionContainingState()
    //     WorkspaceCoord fake_coord;
    //     m_goal_state_id = createState(fake_coord);
    //     m_goal_entry = getState(m_goal_state_id);
    // }

    // SMPL_DEBUG_NAMED("graph", "  goal state has id %d", m_goal_state_id);

    SMPL_DEBUG_NAMED("graph", "initialize environment");
    readGoalRegion();
    return true;
}

bool WorkspaceLatticeZero::readGoalRegion()
{
    XmlRpc::XmlRpcValue xlist;
    if (!m_nh.getParam("start_region/min_limits", xlist)) {
        ROS_WARN("Could not find start region min limits");
        return false;
    }

    if (xlist.size() != 6 + freeAngleCount()) {
        ROS_ERROR("min limits: %d params required, %zu provided", 6 + freeAngleCount(), xlist.size());
        return false;
    }

    for (size_t i = 0; i < xlist.size(); ++i) {
        m_min_ws_limits.push_back(xlist[i]);
    }

    if (!m_nh.getParam("start_region/max_limits", xlist)) {
        ROS_WARN("Could not find start region max limits");
        return false;
    }

    if (xlist.size() != 6 + freeAngleCount()) {
        ROS_ERROR("max limits: %d params required, %zu provided", 6 + freeAngleCount(), xlist.size());
        return false;
    }

    for (size_t i = 0; i < xlist.size(); ++i) {
        m_max_ws_limits.push_back(xlist[i]);
    }

    // normalize [-pi,pi]x[-pi/2,pi/2]x[-pi,pi]
    // center of cell
    WorkspaceCoord limits_coord(6 + freeAngleCount());
    stateWorkspaceToCoord(m_min_ws_limits, limits_coord);
    stateCoordToWorkspace(limits_coord, m_min_ws_limits);

    stateWorkspaceToCoord(m_max_ws_limits, limits_coord);
    stateCoordToWorkspace(limits_coord, m_max_ws_limits);

    // printf("min %f max %f\n", m_min_ws_limits[0], m_max_ws_limits[0]);
    // printf("min %f max %f\n", m_min_ws_limits[1], m_max_ws_limits[1]);
    // printf("min %f max %f\n", m_min_ws_limits[2], m_max_ws_limits[2]);

    // getchar();

    for (size_t i = 0; i < m_min_ws_limits.size(); ++i) {
        if (m_min_ws_limits[i] > m_max_ws_limits[i]) {
            ROS_ERROR("Min limit greater than max limit at index %zu", i);
            return false;
        }
    }

    m_distribution.resize(6 + freeAngleCount());

    for (int i = 0; i < m_distribution.size() ; ++i) {
        m_distribution[i] = std::uniform_real_distribution<double> (m_min_ws_limits[i], m_max_ws_limits[i]);
    }
    return true;
}

bool WorkspaceLatticeZero::projectToPose(int state_id, Eigen::Isometry3d& pose)
{
    // if (state_id == getGoalStateID()) {
    //     pose = goal().pose;
    //     return true;
    // }
    WorkspaceLatticeState* state = getState(state_id);

    double p[6];
    poseCoordToWorkspace(&state->coord[0], &p[0]);

    pose = Eigen::Translation3d(p[0], p[1], p[2]) *
            Eigen::AngleAxisd(p[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(p[3], Eigen::Vector3d::UnitX());
    return true;
}

const WorkspaceState& WorkspaceLatticeZero::extractState(int state_id)
{
    stateCoordToWorkspace(m_states[state_id]->coord, m_workspace_state);
    return m_workspace_state;
}

void WorkspaceLatticeZero::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
    GetSuccs(state_id, preds, costs);
    // SMPL_WARN("GetPreds unimplemented");
}

bool WorkspaceLatticeZero::IsStateValid(int state_id)
{
    // return true;
    WorkspaceLatticeState* entry = getState(state_id);
    // IK
    WorkspaceState workspace_state;
    stateCoordToWorkspace(entry->coord, workspace_state);

    if (!stateWorkspaceToRobot(workspace_state, entry->state)) {
        // ROS_WARN("IK failed");
        return false;
    }
    // SMPL_INFO_STREAM_NAMED("graph.expands", "  seeed state: " << m_ik_seed);
    // SMPL_INFO_STREAM_NAMED("graph.expands", "  robot state: " << entry->state);

    if (!collisionChecker()->isStateValid(
        entry->state, true)) {
        // ROS_WARN("%d : In Collision", state_id);
        return false;
    }

    // no need right?
    // if (!robot()->checkJointLimits(entry->state)) {
    //     return false;
    // }
    return true;
}

bool WorkspaceLatticeZero::IsStateToStateValid(int from_state_id, int to_state_id)
{
    if (!IsStateValid(from_state_id)) {
        return false;
    }

    WorkspaceLatticeState* from_entry = getState(from_state_id);
    WorkspaceLatticeState* to_entry = getState(to_state_id);

    assert((from_entry->state).size() == robot()->jointVariableCount());

// #ifdef tie_breaking
    if (to_entry->state.empty()) {
        WorkspaceState workspace_state;
        stateCoordToWorkspace(to_entry->coord, workspace_state);
        RobotState irstate;
        if (!stateWorkspaceToRobot(workspace_state, to_entry->state)) {
            return false;
        }
    }
// #endif

    if (!collisionChecker()->isStateToStateValid(from_entry->state, to_entry->state)) {
        return false;
    }
    return true;
}

void WorkspaceLatticeZero::PruneRegions()
{
    // prune
    std::vector<region> filtered_regions;
    for (const auto& r1 : *m_regions_ptr) {
        bool keep = true;
        for (const auto& r2 : *m_regions_ptr) {
            double eps = 0.0001;
            auto h = heuristic(0);
            WorkspaceCoord c1, c2;
            stateWorkspaceToCoord(r1.state, c1);
            stateWorkspaceToCoord(r2.state, c2);
            int id1 = createState(c1);
            int id2 = createState(c2);
            int d = h->GetFromToHeuristic(id1, id2);
            if (h->GetFromToHeuristic(id1, id2) == 0) {
                continue;
            }
            if (r2.radius > d + r1.radius) {
                keep = false;
                break;
            }
        }
        if (keep) {
            filtered_regions.push_back(r1);
        }
    }
    ROS_INFO("Original regions: %zu, filtered regions: %zu",
        m_regions_ptr->size(), filtered_regions.size());
    *m_regions_ptr = filtered_regions;
}

bool WorkspaceLatticeZero::IsStateCovered(bool valid, const int state_id)
{
    // Check if in an existing region
    std::vector<region>* regions_ptr;
    if (valid)
        regions_ptr = m_regions_ptr;
    else
        regions_ptr = m_iregions_ptr;

    for (const auto& r : *regions_ptr) {
        WorkspaceCoord coord;
        stateWorkspaceToCoord(r.state, coord);
        int center_state_id = createState(coord);
        RobotHeuristic* h = heuristic(0);   //TODO: Pick the correct heuristic the right way
        int dsum = h->GetFromToHeuristic(state_id, center_state_id);
        // if (valid)
        //     printf("dsum %d radius %u\n", dsum, r.radius);
        if (dsum < r.radius || dsum == 0) {
            return true;
        }
    }
    return false;
}

int WorkspaceLatticeZero::SampleAttractorState(
    WorkspaceState& workspace_state,
    int max_tries)
{
    int attractor_state_id;
    int count = 0;
    while (count < max_tries) {
        count++;

        RobotState joint_state;
        if (!SampleRobotState(joint_state)) {
            continue;
        }
        // WorkspaceState workspace_state;
        WorkspaceCoord workspace_coord;
        stateRobotToWorkspace(joint_state, workspace_state);
        stateWorkspaceToCoord(workspace_state, workspace_coord);
        int attractor_state_id = createState(workspace_coord);

        if (IsStateCovered(true, attractor_state_id)) {
            SMPL_DEBUG_NAMED("graph", "State is covered already on try %d", count);
            continue;
        }

        SMPL_DEBUG_NAMED("graph", "Sampled Attractor State");
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "    workspace state: " << workspace_state);
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "    workspace coord: " << workspace_coord);
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "    joint state    : " << joint_state);

        WorkspaceLatticeState* entry = getState(attractor_state_id);
        entry->state = joint_state;
        m_goal_state_id = attractor_state_id;  // for WorkspaceDistHeuristic
        m_valid_front.insert(entry);

        // set the (modified) goal
        GoalConstraint gc = m_goal;     //may not be required but just in case
        gc.angles = workspace_state;
        gc.type = GoalType::JOINT_STATE_GOAL;
        if (!RobotPlanningSpace::setGoal(gc)) {
            ROS_ERROR("Set new attractor goal failed");
        }

        auto* vis_name = "attractor_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(joint_state, vis_name));
        // m_ik_seed = joint_state;
        ROS_INFO("Sampled attractor %zu on try: %d", m_regions_ptr->size(), count);
        return attractor_state_id;
    }

    return -1;
}

bool WorkspaceLatticeZero::SampleRobotState(RobotState& joint_state)
{
    std::vector<double> workspace_state(6 + freeAngleCount());
    for (int i = 0; i < workspace_state.size() ; ++i) {
        workspace_state[i] = m_distribution[i](m_generator);
    }

    // normalize and project to center
    // normalize_euler_zyx(&workspace_state[3]);
    WorkspaceCoord workspace_coord;
    stateWorkspaceToCoord(workspace_state, workspace_coord);
    stateCoordToWorkspace(workspace_coord, workspace_state);

    if (!stateWorkspaceToRobot(workspace_state, joint_state)) {
        SMPL_DEBUG_NAMED("graph", "Unable to sample robot state: Invalid IK");
        return false;
    }

    // Collision check`
    if (!collisionChecker()->isStateValid(joint_state, true)) {
        SMPL_DEBUG_NAMED("graph", "Unable to sample robot state: Collision");
        return false;
    }

    SMPL_DEBUG_NAMED("graph", "Sampled State");
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    workspace state: " << workspace_state);
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    workspace coord: " << workspace_coord);
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    joint state    : " << joint_state);

    return true;
}

bool WorkspaceLatticeZero::SearchForValidIK(const GoalConstraint goal, std::vector<double>& angles)
{
    double fa_init = m_min_ws_limits[6]; // assuming there is only one redundant joint
    std::vector<double> seed(6 + freeAngleCount());
    for (double fa = fa_init; fa <= m_max_ws_limits[6]; ++fa) {
        seed[6] = fa;
        if (stateWorkspaceToRobot(goal.angles, seed, angles)) {
            return true;
        }
    }
    return false;
}

int WorkspaceLatticeZero::FindRegionContainingState(const RobotState& joint_state)
{
    WorkspaceState workspace_state;
    WorkspaceCoord workspace_coord;
    stateRobotToCoord(joint_state, workspace_coord);
    stateCoordToWorkspace(workspace_coord, workspace_state);
    int query_state_id = createState(workspace_coord);

    // Collision check`
    if (!collisionChecker()->isStateValid(joint_state, true)) {
        ROS_ERROR("Goal state is in collision");
        return -1;
    }

    bool covered = false;
    int reg_idx = 0;
    WorkspaceState goal_state;
    for (const auto& r : *m_regions_ptr) {
        // WorkspaceState c_workspace_state;
        WorkspaceCoord workspace_coord;
        stateWorkspaceToCoord(r.state, workspace_coord);
        int attractor_state_id = createState(workspace_coord);
        RobotHeuristic* h = heuristic(0);
        int dsum = h->GetFromToHeuristic(query_state_id, attractor_state_id);
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "    query state:     " << workspace_state);
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "    attractor state: " << r.state);

        if (dsum < r.radius || dsum == 0) {
            // printf("dsum %d radius %u id1 %d id2 %d\n", dsum, r.radius, query_state_id, attractor_state_id);
            // ROS_INFO("Covered try %d", count);
            goal_state = r.state;
            covered = true;
            break;
        }
        reg_idx++;
    }

    if (covered) {
        SMPL_DEBUG_NAMED("graph", "Attractor State of Containing Region %d", reg_idx);
        return reg_idx;
    }
    else {
        SMPL_INFO_STREAM_NAMED("graph.expands", "  start workspace_state: " << workspace_state);
        return -1;
    }
}


bool WorkspaceLatticeZero::IsRobotStateInGoalRegion(const RobotState& state)
{
    WorkspaceState workspace_state;
    stateRobotToWorkspace(state, workspace_state);
    // normalize_euler_zyx(&workspace_state[3]);
    WorkspaceCoord coord;
    stateWorkspaceToCoord(workspace_state, coord);
    stateCoordToWorkspace(coord, workspace_state);
    return IsWorkspaceStateInGoalRegion(workspace_state);
}

bool WorkspaceLatticeZero::IsWorkspaceStateInGoalRegion(const WorkspaceState& state)
{
    double eps = 0.0001;
    for (int i = 0; i < state.size(); ++i) {
        if (state[i] < m_min_ws_limits[i] - eps || state[i] > m_max_ws_limits[i] + eps) {
            SMPL_DEBUG_NAMED("graph", "violates start region limits: %d, val: %f, min limit: %f, max limit: %f", i, state[i], m_min_ws_limits[i], m_max_ws_limits[i]);
            return false;
        }
    }
    return true;
}

void WorkspaceLatticeZero::PruneCoveredStates(std::vector<WorkspaceState>& workspace_states)
{
    std::vector<WorkspaceState> pruned_states;
    for (const auto& s : workspace_states) {
        WorkspaceCoord coord;
        stateWorkspaceToCoord(s, coord);
        int state_id = createState(coord);
        if (!IsStateCovered(true, state_id) && !IsStateCovered(false, state_id)) {
            pruned_states.push_back(s);
        }
    }
    workspace_states = pruned_states;
}

void WorkspaceLatticeZero::FillFrontierLists(
    const std::vector<int>& state_ids)
{
    for (const auto& state_id : state_ids) {
        auto entry = getState(state_id);
        if (IsStateValid(state_id)) {
            m_valid_front.insert(entry);
        }
        else {
            m_invalid_front.insert(entry);
        }
    }
}

void WorkspaceLatticeZero::GetWorkspaceState(const int state_id, WorkspaceState& workspace_state)
{
    auto entry = getState(state_id);
    stateCoordToWorkspace(entry->coord, workspace_state);
}

void WorkspaceLatticeZero::GetJointState(const int state_id, RobotState& joint_state)
{
    auto entry = getState(state_id);
    joint_state = entry->state;
}

int WorkspaceLatticeZero::SetAttractorState()
{
    auto it = m_valid_front.begin();
    auto entry = *it;
    m_valid_front.erase(it);

    WorkspaceState workspace_state;
    stateCoordToWorkspace(entry->coord, workspace_state);
    if (entry->state.empty()) {
        stateWorkspaceToRobot(workspace_state, entry->state);
    }

    int attractor_state_id =  createState(entry->coord);
    m_goal_state_id = attractor_state_id;  // for WorkspaceDistHeuristic

    // set the (modified) goal
    GoalConstraint gc = m_goal;     //may not be required but just in case
    gc.angles = workspace_state;
    gc.type = GoalType::JOINT_STATE_GOAL;
    if (!RobotPlanningSpace::setGoal(gc)) {
        ROS_ERROR("Set new attractor state goal failed");
    }

    // if (m_regions_ptr->size() >= 1534) {
    //     SMPL_INFO_STREAM_NAMED("graph.expands", "    attractor state: " << workspace_state);
    // }

    // auto* vis_name = "attractor_config";
    // SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(joint_state, vis_name));
    // m_ik_seed = state;
    // ROS_INFO("New attractor state set");

    return attractor_state_id;
}

int WorkspaceLatticeZero::SetInvalidStartState()
{
    auto it = m_invalid_front.begin();
    auto entry = *it;
    m_invalid_front.erase(it);

    WorkspaceState workspace_state;
    stateCoordToWorkspace(entry->coord, workspace_state);
    if (entry->state.empty()) {
        stateWorkspaceToRobot(workspace_state, entry->state);
    }

    SMPL_DEBUG_NAMED("graph", "Invalid Start State");
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    workspace state: " << workspace_state);
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    workspace coord: " << entry->coord);
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    joint state    : " << entry->state);

    int iv_state_id =  createState(entry->coord);
    m_goal_state_id = iv_state_id;  // for WorkspaceDistHeuristic

    // for heuristic
    // set the (modified) goal
    GoalConstraint gc = m_goal;     //may not be required but just in case
    gc.angles = workspace_state;
    gc.type = GoalType::JOINT_STATE_GOAL;
    if (!RobotPlanningSpace::setGoal(gc)) {
        ROS_ERROR("Set new iv state goal failed");
    }

    // because invalid
    // auto* vis_name = "iv_config";
    // SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(state, vis_name));
    // m_ik_seed = state;
    // ROS_INFO("Invalid Start State set");
    return iv_state_id;
}

void WorkspaceLatticeZero::PassRegions(
    std::vector<region>* regions_ptr,
    std::vector<region>* iregions_ptr)
{
    m_regions_ptr = regions_ptr;
    m_iregions_ptr = iregions_ptr;
}

void WorkspaceLatticeZero::VisualizePoint(int state_id, std::string type)
{
#if 0
    WorkspaceLatticeState* entry = getState(state_id);
    int hue = 0;
    if (type == "greedy")
        hue = 100;
    else if (type == "attractor")
        hue = 199;
    else if (type == "exited")
        hue = 299;

    WorkspaceState ws_parent;
    stateCoordToWorkspace(entry->coord, ws_parent);
    // SMPL_INFO_STREAM_NAMED("graph.expands", "  state: " << ws_parent);
    auto vis_name = type;
    auto marker = visual::MakeSphereMarker(ws_parent[0],
                                           ws_parent[1],
                                           ws_parent[2],
                                           m_res[0]/2,
                                           hue,
                                           m_viz_frame_id,
                                           vis_name,
                                           m_vis_id);
    SV_SHOW_INFO_NAMED(vis_name, marker);
    m_vis_id++;
    getchar();
#endif
}

bool WorkspaceLatticeZero::IsQueryCovered(
    const RobotState& full_start_state,
    const GoalConstraint& goal)
{
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    query start state  : " << full_start_state);
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "    regions start state: " << (*m_regions_ptr).front().start);

    double eps = 0.25;
    for (size_t i = 0; i < full_start_state.size(); ++i) {
        if (fabs(full_start_state[i] - (*m_regions_ptr).front().start[i]) > eps) {
            ROS_WARN("ZTP: start state is not preprocessed, index %d is different", i);
            return false;
        }
    }

    if (!IsRobotStateInGoalRegion(goal.angles)) {
        ROS_WARN("ZTP: goal state is not covered in goal region");
        return false;
    }

    // check if the discrete goal is in collision
    WorkspaceState ws_state;
    RobotState discrete_state;
    stateRobotToWorkspace(goal.angles, ws_state);
    stateWorkspaceToRobot(ws_state, discrete_state);
    if (!collisionChecker()->isStateValid(discrete_state, true)) {
        ROS_WARN("The discretized goal state in collision");
        return false;
    }

    return true;
}

bool WorkspaceLatticeZero::setGoal(const GoalConstraint& goal)
{
    m_goal = goal;

    bool res = false;
    if (goal.type == GoalType::XYZ_RPY_GOAL) {
        return setGoalPose(goal);
    } else {
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "  goal: " << goal.angles);
        WorkspaceCoord goal_coord;
        stateWorkspaceToCoord(goal.angles, goal_coord);
        GoalConstraint gc;
        gc.angles = goal.angles;
        gc.type = GoalType::JOINT_STATE_GOAL;
        if (!RobotPlanningSpace::setGoal(gc)) {
            ROS_ERROR("Set new goal goal failed");
            return false;
        }
        m_goal_state_id = createState(goal_coord);

        // set ik seed
        // m_ik_seed = goal.angles;
    }

    return true;
}

int WorkspaceLatticeZero::getStartStateID() const
{
    if (m_start_state_id >= 0 && m_goal_state_id >= 0) {
        WorkspaceLatticeState* start_state = getState(m_start_state_id);
        WorkspaceState cont_state;
        stateCoordToWorkspace(start_state->coord, cont_state);
        if (isGoal(cont_state)) {
            return m_goal_state_id;
        }
    }
    return m_start_state_id;
}

bool WorkspaceLatticeZero::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    path.clear();

    if (ids.empty()) {
        return true;
    }

    if (ids.size() == 1) {
        const int state_id = ids[0];
        if (state_id == getGoalStateID()) {
            const WorkspaceLatticeState* entry = getState(m_start_state_id);
            if (!entry) {
                SMPL_ERROR_NAMED("graph", "Failed to get state entry for state %d", m_start_state_id);
                return false;
            }
            path.push_back(entry->state);
        } else {
            const WorkspaceLatticeState* entry = getState(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED("graph", "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
        return true;
    }

    WorkspaceLatticeState* start_entry = getState(ids[0]);
    WorkspaceState workspace_state;
    stateCoordToWorkspace(start_entry->coord, workspace_state);
    if (start_entry->state.empty()) {
        if (!stateWorkspaceToRobot(workspace_state, start_entry->state)) {
            ROS_ERROR("ZTP query: failed to find ik for start state");
        }
    }
    path.push_back(start_entry->state);

    for (size_t i = 1; i < ids.size(); ++i) {
        const int prev_id = ids[i - 1];
        const int curr_id = ids[i];

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED("graph", "cannot determine goal state successors during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            // TODO: variant of get succs that returns unique state ids
            WorkspaceLatticeState* prev_entry = getState(prev_id);
            std::vector<Action> actions;
            m_actions->apply(*prev_entry, actions);

            WorkspaceLatticeState* best_goal_entry = nullptr;
            int best_cost = std::numeric_limits<int>::max();

            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                const WorkspaceState& final_state = action.back();
                if (!isGoal(final_state)) {
                    continue;
                }

                if (!checkAction(prev_entry->state, action)) {
                    continue;
                }

                WorkspaceCoord goal_coord;
                stateWorkspaceToCoord(final_state, goal_coord);

                int goal_id = createState(goal_coord);
                WorkspaceLatticeState* goal_state = getState(goal_id);

                // shouldn't have created a new state, so no need to set the
                // continuous state counterpart
                assert(goal_state->state.size() == robot()->jointVariableCount());

                best_cost = 30; // Hardcoded primitive value in GetSuccs
                best_goal_entry = goal_state;
                break;
            }

            if (!best_goal_entry) {
                SMPL_ERROR_NAMED("graph", "failed to find valid goal successor during path extraction");
                return false;
            }

            WorkspaceState workspace_state;
            stateCoordToWorkspace(best_goal_entry->coord, workspace_state);
            if (best_goal_entry->state.empty()) {
                if (!stateWorkspaceToRobot(workspace_state, best_goal_entry->state)) {
                    ROS_ERROR("ZTP query: failed to find ik for goal state");
                }
            }
            path.push_back(best_goal_entry->state);
        } else {
            WorkspaceLatticeState* state_entry = getState(curr_id);
            WorkspaceState workspace_state;
            stateCoordToWorkspace(state_entry->coord, workspace_state);
            if (state_entry->state.empty()) {
                if (!stateWorkspaceToRobot(workspace_state, state_entry->state)) {
                    ROS_ERROR("ZTP query: failed to find ik for state");
                }
            }
            path.push_back(state_entry->state);
        }
    }

    return true;
}

Extension* WorkspaceLatticeZero::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<WorkspaceLatticeZero>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PoseProjectionExtension>() ||
        class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>())
    {
        return this;
    }
    return nullptr;
}

void WorkspaceLatticeZero::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    // clear the successor arrays
    succs->clear();
    costs->clear();

    SMPL_DEBUG_NAMED("graph.expands", "Expand state %d", state_id);

    // goal state should be absorbing
    // if (state_id == m_goal_state_id) {
    //     return;
    // }

    WorkspaceLatticeState* parent_entry = getState(state_id);

    assert(parent_entry);
    assert(parent_entry->coord.size() == m_dof_count);

    SMPL_DEBUG_STREAM_NAMED("graph.expands", "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED("graph.expands", "  state: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));
    // getchar();

    // if (m_regions_ptr->size() > 1534) {
    //     WorkspaceState ws_parent;
    //     stateCoordToWorkspace(parent_entry->coord, ws_parent);
    //     SMPL_INFO_STREAM_NAMED("graph.expands", "  state: " << ws_parent);
    // }

#if 0
    int hue;
    bool valid = true;
    if (!parent_entry->state.empty()) {
        if (!collisionChecker()->isStateValid(parent_entry->state, true)) {
            SMPL_DEBUG_NAMED("graph", "State in collision on try");
            valid = false;
        }
    }
    else {
        if (!IsStateValid(createState(parent_entry->coord))) {
            valid = false;
        }
    }
    if (valid) {
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));
        hue = 30;
    }
    else {
        hue = 0;
    }
    WorkspaceState ws_parent;
    stateCoordToWorkspace(parent_entry->coord, ws_parent);
    // SMPL_INFO_STREAM_NAMED("graph.expands", "  state: " << ws_parent);
    vis_name = "workspace_expansion";
    auto marker = visual::MakeSphereMarker(ws_parent[0],
                                           ws_parent[1],
                                           ws_parent[2],
                                           m_res[0]/2,
                                           hue,
                                           m_viz_frame_id,
                                           vis_name,
                                           m_vis_id);
    SV_SHOW_INFO_NAMED(vis_name, marker);
    m_vis_id++;
    getchar();
    // ros::Duration(0.01).sleep();
#endif

    std::vector<Action> actions;
    m_actions->apply(*parent_entry, actions);

    SMPL_DEBUG_NAMED("graph.expands", "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED("graph.expands", "    action %zu", i);
        SMPL_DEBUG_NAMED("graph.expands", "      waypoints: %zu", action.size());

        RobotState final_rstate;
        if (m_search_mode == REACHABILITY) {
            if (!checkActionPreprocessing(parent_entry->state, action, &final_rstate)) {
                continue;
            }
        }
        // else {
        //     if (!checkAction(parent_entry->state, action, &final_rstate)) {
        //         continue;
        //     }
        // }

        const WorkspaceState& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_id = createState(succ_coord);
        WorkspaceLatticeState* succ_state = getState(succ_id);
        succ_state->state = final_rstate;

        // check if this state meets the goal criteria
        const bool is_goal_succ = false;
        // const bool is_goal_succ = isGoal(final_state);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_id);
        }

        const int edge_cost = 30;
        costs->push_back(edge_cost);

        SMPL_DEBUG_NAMED("graph.expands", "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED("graph.expands", "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED("graph.expands", "        cost: %5d", edge_cost);
    }
}

void WorkspaceLatticeZero::ClearStates()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();
    m_state_to_id.clear();
    m_states.shrink_to_fit();
}

bool WorkspaceLatticeZero::isGoal(const WorkspaceState& state) const
{
    // check position
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL: {
        // SMPL_WARN_ONCE("WorkspaceLattice joint-space goals not implemented");
        WorkspaceCoord coord;
        stateWorkspaceToCoord(state, coord);
        WorkspaceLatticeState s;
        s.coord = coord;
        auto sit = m_state_to_id.find(&s);
        int state_id;
        if (sit != m_state_to_id.end()) {
            state_id = sit->second;
        }
        return state_id == m_goal_state_id;
    }
    case GoalType::XYZ_RPY_GOAL: {
        double dx = std::fabs(state[0] - goal().pose.translation()[0]);
        double dy = std::fabs(state[1] - goal().pose.translation()[1]);
        double dz = std::fabs(state[2] - goal().pose.translation()[2]);
        if (dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                auto now = clock::now();
                double time_to_goal_region =
                        std::chrono::duration<double>(now - m_t_start).count();
                m_near_goal = true;
                SMPL_INFO("search is at the goal position after %0.3f sec", time_to_goal_region);
            }

            Eigen::Quaterniond qg(goal().pose.rotation());
            Eigen::Quaterniond q(
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX()));

            if (q.dot(qg) < 0.0) {
                qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

//            const double theta = angles::normalize_angle(Eigen::AngleAxisd(qg.conjugate() * q).angle());
            const double theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            if (theta < goal().rpy_tolerance[0]) {
                return true;
            }
        }
        return false;
    }   break;
    case GoalType::XYZ_GOAL: {
        SMPL_WARN_ONCE("WorkspaceLatticeZero xyz goals not implemented");
        return false;
    }   break;
    default:
        return false;
    }
}

bool WorkspaceLatticeZero::checkActionPreprocessing(
    const RobotState& state,
    const Action& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    WorkspaceState end_state;
    end_state = action[action.size() - 1];

    SMPL_DEBUG_STREAM_NAMED("graph.expands", "  end state: " << end_state);
    if (!IsWorkspaceStateInGoalRegion(end_state)) {
        violation_mask |= 0x00000001;
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

bool WorkspaceLatticeZero::checkAction(
    const RobotState& state,
    const Action& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        const WorkspaceState& istate = action[widx];

        SMPL_DEBUG_STREAM_NAMED("graph.expands", "        " << widx << ": " << istate);

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, irstate)) {
            SMPL_DEBUG_NAMED("graph.expands", "         -> failed to find ik solution");
            violation_mask |= 0x00000001;
            break;
        }

        wptraj.push_back(irstate);

        // no need
        // if (!robot()->checkJointLimits(irstate)) {
        //     SMPL_DEBUG_NAMED("graph.expands", "        -> violates joint limits");
        //     violation_mask |= 0x00000002;
        //     break;
        // }

    }

    if (violation_mask) {
        return false;
    }

#ifdef tie_breaking
    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (!collisionChecker()->isStateToStateValid(state, wptraj[0])) {
        SMPL_DEBUG_NAMED("graph.expands", "        -> path to first waypoint in collision");
        // SMPL_INFO_STREAM_NAMED("graph.expands", "  attractor coord   : " << action[0]);
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    for (size_t widx = 1; widx < wptraj.size(); ++widx) {
        const RobotState& prev_istate = wptraj[widx - 1];
        const RobotState& curr_istate = wptraj[widx];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate)) {
            SMPL_DEBUG_NAMED("graph.expands", "        -> path between waypoints in collision");
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }
#endif

    if (final_rstate) {
        *final_rstate = wptraj.back();
    }
    return true;
}

} // namespace smpl
