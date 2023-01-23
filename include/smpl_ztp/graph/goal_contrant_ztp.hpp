//
// Created by itamar on 1/22/23.
//

#ifndef SMPL_ZTP_GOAL_CONTRANT_ZTP_HPP
#define SMPL_ZTP_GOAL_CONTRANT_ZTP_HPP

#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/spatial.h>
#include <smpl/types.h>

namespace smpl {
    struct GoalConstraintZTP : public GoalConstraint {

        WorkspaceState ws_state;
    };
}


#endif //SMPL_ZTP_GOAL_CONTRANT_ZTP_HPP

