#ifndef DLSC_GC_PLANNER_GOAL_OPTIMIZER_HPP
#define DLSC_GC_PLANNER_GOAL_OPTIMIZER_HPP

/**
 * ===================================================================================
 * [THEORY: Subgoal Optimization - Park et al. 2023/2025, Section 5]
 * ===================================================================================
 *
 * The GoalOptimizer finds the INTERMEDIATE GOAL (subgoal) that guides the agent
 * toward the discrete waypoint from MAPF while respecting continuous collision
 * constraints.
 *
 * PROBLEM FORMULATION:
 * Given:
 * - current_goal_point: Agent's current goal from previous planning iteration
 * - next_waypoint: Target waypoint from grid-based MAPF planner
 *
 * Find:
 * - Optimal subgoal g* on the line segment from current_goal to next_waypoint
 *   that is closest to next_waypoint while satisfying all collision constraints
 *
 * OPTIMIZATION (Line Search):
 *   minimize    t
 *   subject to: g = (current_goal - next_waypoint) * t + next_waypoint
 *               g ∈ SFC (Static Flight Corridor)
 *               g satisfies LSC for all non-dynamic obstacles
 *               t ∈ [0, 1]
 *
 * INTERPRETATION:
 * - t = 0: subgoal = next_waypoint (fully reached waypoint)
 * - t = 1: subgoal = current_goal (no progress toward waypoint)
 * - t ∈ (0,1): subgoal is between current and next waypoint
 *
 * This optimization bridges the gap between:
 * - Discrete MAPF planning (deadlock-free waypoints)
 * - Continuous trajectory optimization (dynamically feasible paths)
 *
 * [QUADRUPED ADAPTATION NOTE]:
 * For 2D navigation, the SFC becomes a 2D polygon (convex region).
 * LSC constraints become 2D half-planes.
 * ===================================================================================
 */

#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <collision_constraints.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CPLEX
#include <ilcplex/ilocplex.h>

namespace MATP {
    /**
     * Subgoal optimizer using QP to find closest feasible point to next waypoint.
     */
    class GoalOptimizer {
    public:
        GoalOptimizer(const Param& param, const Mission& mission);

        /**
         * Solve the subgoal optimization problem.
         *
         * @param agent Current agent state
         * @param constraints Collision constraints (SFC, LSC) from constraint generator
         * @param current_goal_point Agent's current goal (from previous iteration)
         * @param next_waypoint Target waypoint from MAPF planner
         * @return Optimal subgoal point that satisfies all constraints
         */
        point3d solve(const Agent& agent,
                      const CollisionConstraints& constraints,
                      const point3d &current_goal_point,
                      const point3d &next_waypoint);

    private:
        Param param;
        Mission mission;

        /** Build the QP model with objective and constraints */
        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                           const Agent& agent, const CollisionConstraints& constraints,
                           const point3d &current_goal_point,
                           const point3d &next_waypoint);
    };
}


#endif //DLSC_GC_PLANNER_GOAL_OPTIMIZER_HPP
