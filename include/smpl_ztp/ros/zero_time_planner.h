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

#ifndef SMPL_ZERO_TIME_PLANNER_H
#define SMPL_ZERO_TIME_PLANNER_H

// system includes
#include <set>

// project includes
#include <smpl/graph/manip_lattice.h>
#include <smpl/search/arastar.h>
#include <smpl/types.h>
#include <smpl_ztp/graph/goal_contrant_ztp.hpp>

#include <smpl_ztp/graph/workspace_lattice_zero.h>
#include <smpl_ztp/search/arastar_zero.h>

// OMPL +  Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

namespace smpl {

class ZeroTimePlanner
{
public:

	ZeroTimePlanner(
		ManipLattice* manip_space,
		WorkspaceLatticeZero* task_space,
		ARAStar* planner,
	    ARAStarZero* planner_zero);

	~ZeroTimePlanner();

    bool isQueryCovered(
        const RobotState& full_start_state,
        const GoalConstraint& goal);

//    void setStartAndGoal(
//        const RobotState& start_state,
//        const GoalConstraint& goal);

    void setStartAndGoal(
            const RobotState& start_state,
            const GoalConstraintZTP& goal);

    void PreProcess(const RobotState& full_start_state);
    void Query(std::vector<RobotState>& path);

    void GraspQuery(std::vector<RobotState>& path, std::string grasp_dir = "/home/itamar/work/code/ros/assembly_ws/src/smpl_ztp/src/ros/data");

    void getLimits();

private:

    void InitMoveitOMPL();
    bool PlanPathFromStartToAttractorOMPL(const RobotState& attractor, std::vector<RobotState>& path);
    bool PlanPathFromStartToAttractorSMPL(const RobotState& attractor, std::vector<RobotState>& path);

    ros::NodeHandle m_nh;
    std::string m_pp_planner;

    RobotState m_start_state;
    GoalConstraintZTP m_goal;

    ManipLattice* m_manip_space;
    WorkspaceLatticeZero* m_task_space;

    // valid/invalid uncovered frontier states
    // std::set<WorkspaceState> m_valid_front;
    // std::set<WorkspaceState> m_invalid_front;

    // attractors failed to reach goal
    std::set<WorkspaceState> m_bad_attractors;

    ARAStar* m_planner;
    ARAStarZero* m_planner_zero;

    std::vector<region> m_regions;
    std::vector<region> m_iregions;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_group;
    moveit::planning_interface::PlanningSceneInterface m_planning_scene_interface;

    void WriteRegions(std::string path = "/home/itamar/work/code/ros/assembly_ws/src/smpl_ztp/src/ros/data/myfile3.dat");

    void ReadRegions(std::string path = "/home/itamar/work/code/ros/assembly_ws/src/smpl_ztp/src/ros/data/myfile2.dat");

};

} // namespace smpl

#endif
