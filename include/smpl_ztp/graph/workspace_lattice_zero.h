////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush, Fahad Islam
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

#ifndef SMPL_WORKSPACE_LATTICE_ZERO_H
#define SMPL_WORKSPACE_LATTICE_ZERO_H

// standard includes
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/serialization/vector.hpp>
#include <set>
#include <random>

// system includes
#include <ros/ros.h>

// project includes
#include <smpl/graph/workspace_lattice.h>

namespace smpl {

struct region
{
    friend class boost::serialization::access;
    RobotState start;
    unsigned int radius;
    WorkspaceState state;
    std::vector<RobotState> path;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & start;
        ar & radius;
        ar & state;
        ar & path;
    }
};

enum Modes
{
    REACHABILITY = 0,
    QUERY
};

/// \class Discrete state lattice representation representing a robot as the
///     pose of one of its links and all redundant joint variables
class WorkspaceLatticeZero : public WorkspaceLattice
{
public:

    ~WorkspaceLatticeZero();

    /// \name Reimplemented Public Functions from WorkspaceLatticeBase
    ///@{
    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const Params& params,
        WorkspaceLatticeActionSpace* actions);
    ///@}

    bool readGoalRegion();

    void ClearStates();

    // /// \name Reimplemented Public Functions from RobotPlanningSpace
    // ///@{
    bool setGoal(const GoalConstraint& goal) override;
    // ///@}

    // used in Workspace Distance heuristic
    bool projectToPose(int state_id, Eigen::Isometry3d& pose) override;

    // just a copy to call this class's isGoal
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;

    // copy beto call this class's isGoal
    int getStartStateID() const override;

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;

    /// \name Required Public Functions from ExtractRobotStateExtension
    ///@{
    const WorkspaceState& extractState(int state_id) override;
    ///@}

    // reachability
    bool IsStateValid(int state_id);
    bool IsStateToStateValid(int from_state_id, int to_state_id);
    void PruneRegions();

    bool IsStateCovered(bool valid, const int state_id);

    int SampleAttractorState(
        WorkspaceState& workspace_state,
        int max_tries);

    bool SampleRobotState(RobotState& joint_state);

    bool SearchForValidIK(const GoalConstraint goal, std::vector<double>& angles);

    int FindRegionContainingState(const RobotState& joint_state);

    bool IsRobotStateInGoalRegion(const RobotState& joint_state);
    bool IsWorkspaceStateInGoalRegion(const WorkspaceState& workspace_state);

    void UpdateSearchMode(int search_mode){m_search_mode = search_mode;};

    // convergence
    void PruneCoveredStates(std::vector<WorkspaceState>& states);
    void FillFrontierLists(
        const std::vector<int>& state_ids);
    void GetWorkspaceState(const int state_id, WorkspaceState& workspace_state);
    void GetJointState(const int state_id, RobotState& joint_state);
    int SetAttractorState();
    int SetInvalidStartState();
    void PassRegions(
        std::vector<region>* regions_ptr,
        std::vector<region>* iregions_ptr);
    void VisualizePoint(int state_id, std::string type);
    bool IsQueryCovered(
        const RobotState& full_start_state,
        const GoalConstraint& goal);

    void getLimits();

    // valid/invalid uncovered frontier states
    std::set<WorkspaceLatticeState*> m_valid_front;
    std::set<WorkspaceLatticeState*> m_invalid_front;
private:

    // reachability
    ros::NodeHandle m_nh;
    std::vector<double> m_min_ws_limits;
    std::vector<double> m_max_ws_limits;
    std::vector<std::uniform_real_distribution<double>> m_distribution;
    std::default_random_engine m_generator;
    GoalConstraint m_goal;
    WorkspaceState m_workspace_state;
    RobotState m_ik_seed;
    std::vector<region>* m_regions_ptr;     // be careful
    std::vector<region>* m_iregions_ptr;

    //query
    int m_vis_id = 0;
    int m_search_mode;

    bool initMotionPrimitives();

    bool checkActionPreprocessing(
        const RobotState& state,
        const Action& action,
        RobotState* final_rstate = nullptr);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        RobotState* final_rstate = nullptr);

    // used in path extraction
    bool isGoal(const WorkspaceState& state) const;
};

} // namespace smpl

#endif

