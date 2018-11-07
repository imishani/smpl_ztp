////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush, Fahad Islam

#include <smpl_ztp/heuristic/workspace_dist_heuristic.h>

// standard includes
#include <cmath>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/graph/workspace_lattice_base.h>

namespace smpl {

static const char* LOG = "heuristic.workspace_dist";

bool WorkspaceDistHeuristic::init(RobotPlanningSpace* space)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_ers = space->getExtension<ExtractRobotStateExtension>();
    return true;
}

double WorkspaceDistHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

double WorkspaceDistHeuristic::getMetricStartDistance(double x, double y, double z)
{
    return 0.0;
}

Extension* WorkspaceDistHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int WorkspaceDistHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }
    if (!m_ers) {
        return 0;
    }
    if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL) {
        SMPL_WARN_ONCE("Workspace Distance Heuristic can only compute goal heuristics for Joint Goals");
        return 0;
    }

    const RobotState& goal_state = planningSpace()->goal().angles;
    const RobotState& state = m_ers->extractState(state_id);
    assert(goal_state.size() == state.size());

    return (int)(FIXED_POINT_RATIO * computeJointDistance(state, goal_state));
}

int WorkspaceDistHeuristic::GetStartHeuristic(int state_id)
{
    if (!m_ers) {
        return 0;
    }

    if (state_id == planningSpace()->getStartStateID()) {
        return 0;
    }

    const RobotState& s = planningSpace()->startState();
    const RobotState& t = m_ers->extractState(state_id);
    return (int)(FIXED_POINT_RATIO * computeJointDistance(s, t));
}

int WorkspaceDistHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (!m_ers) {
        return 0;
    }

    // if (from_id == planningSpace()->getGoalStateID()) {
    //     printf("here1 %d %d\n", from_id, to_id);
    //     const RobotState s = planningSpace()->goal().angles;
    //     const RobotState t = m_ers->extractState(to_id);
    //     return (int)(FIXED_POINT_RATIO * computeJointDistance(s, t));
    // } else if (to_id == planningSpace()->getGoalStateID()) {
    //     printf("here2 %d %d\n", from_id, to_id);
    //     const RobotState s = m_ers->extractState(from_id);
    //     const RobotState t = planningSpace()->goal().angles;
    //     return (int)(FIXED_POINT_RATIO * computeJointDistance(s, t));
    // } else {
        // printf("here3 %d %d\n", from_id, to_id);
        const RobotState s = m_ers->extractState(from_id);
        const RobotState t = m_ers->extractState(to_id);
        return (int)(FIXED_POINT_RATIO * computeJointDistance(s, t));
    // }
}

double WorkspaceDistHeuristic::computeJointDistance(
    const RobotState& s,
    const RobotState& t) const
{
    // printf("s %f %f %f %f %f %f %f\n", s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
    // printf("t %f %f %f %f %f %f %f\n", t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
    double dsum = 0.0;
    for (size_t i = 0; i < s.size(); ++i) {
        double dj = (s[i] - t[i]);
        dsum += dj * dj;
    }
    return std::sqrt(dsum);
}

} // namespace smpl
