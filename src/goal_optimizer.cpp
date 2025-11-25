#include "goal_optimizer.hpp"

namespace MATP {
    GoalOptimizer::GoalOptimizer(const Param &_param, const Mission &_mission)
            : param(_param), mission(_mission) {}

    /**
     * ===================================================================================
     * [THEORY: Subgoal Line Search - Park et al. 2023/2025]
     * ===================================================================================
     *
     * Finds the optimal subgoal on the line from current_goal to next_waypoint.
     *
     * OPTIMIZATION PROBLEM:
     *   minimize    t
     *   subject to: g(t) = (current_goal - next_waypoint) * t + next_waypoint
     *               g(t) ∈ SFC (static obstacle free region)
     *               g(t) satisfies LSC for all neighboring agents
     *               t ∈ [0, 1]
     *
     * SOLUTION INTERPRETATION:
     *   t* = 0: Fully reached waypoint (best case)
     *   t* = 1: Cannot progress toward waypoint (blocked)
     *   t* ∈ (0,1): Partial progress toward waypoint
     *
     * The subgoal becomes the "goal" for the subsequent trajectory optimization
     * step, guiding the agent toward the MAPF waypoint while respecting safety.
     * ===================================================================================
     */
    point3d GoalOptimizer::solve(const Agent& agent,
                                 const CollisionConstraints& constraints,
                                 const point3d &current_goal_point,
                                 const point3d &next_waypoint) {

        /** If already at waypoint, return immediately */
        if(current_goal_point.distance(next_waypoint) < SP_EPSILON_FLOAT){
            return next_waypoint;
        }

        point3d goal;
        IloEnv env;
        IloCplex cplex(env);
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);

        /** Configure CPLEX solver */
        cplex.setParam(IloCplex::Param::Threads, 6);

        /** Build QP model with objective and constraints */
        populatebyrow(model, var, con, agent, constraints, current_goal_point, next_waypoint);
        cplex.extract(model);

        std::string QPmodel_path = param.package_path + "/log/QPmodel_goalOpt.lp";
        std::string conflict_path = param.package_path + "/log/conflict_goalOpt.lp";
        if (param.log_solver) {
            cplex.exportModel(QPmodel_path.c_str());
        } else {
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }

        /** Solve the QP and extract optimal subgoal */
        try {
            IloBool success = cplex.solve();

            /** Compute subgoal: g = (current - next) * t* + next */
            IloNumArray vals(env);
            cplex.getValues(vals, var);
            goal = (current_goal_point - next_waypoint) * vals[0] + next_waypoint;
            env.end();
        }
        catch (IloException &e) {
            bool numerical_error = true;
            if (param.world_use_octomap) {
                Box sfc = constraints.getSFC(param.M - 1);
                if(not sfc.isPointInBox(current_goal_point)){
                    numerical_error = false;
                }
            }

            for (size_t oi = 0; oi < constraints.getObsSize(); oi++) {
                if(constraints.isDynamicObstacle(oi)){
                    continue;
                }

                LSC lsc = constraints.getLSC(oi, param.M - 1, param.n);
                if (lsc.normal_vector.norm() < SP_EPSILON_FLOAT) {
                    continue;
                }

                double delta = lsc.normal_vector.dot(current_goal_point - lsc.obs_control_point) - lsc.d;
                if(delta < -SP_EPSILON_FLOAT){
                    numerical_error = false;
                }
            }

            if(numerical_error){
                goal = current_goal_point;
            } else {
                cplex.exportModel(QPmodel_path.c_str());
                if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
                    (cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded)) {
                    ROS_ERROR_STREAM(
                            "[GoalOptimizer] CPLEX No solution at mav " << agent.id
                                                                        << ", starting Conflict refinement");
                    IloConstraintArray infeas(env);
                    IloNumArray preferences(env);

                    infeas.add(con);
                    for (IloInt i = 0; i < var.getSize(); i++) {
                        if (var[i].getType() != IloNumVar::Bool) {
                            infeas.add(IloBound(var[i], IloBound::Lower));
                            infeas.add(IloBound(var[i], IloBound::Upper));
                        }
                    }

                    for (IloInt i = 0; i < infeas.getSize(); i++) {
                        preferences.add(1.0);  // User may wish to assign unique preferences
                    }

                    if (cplex.refineConflict(infeas, preferences)) {
                        IloCplex::ConflictStatusArray conflict = cplex.getConflict(infeas);
                        env.getImpl()->useDetailedDisplay(IloTrue);
                        std::cout << "Conflict :" << std::endl;
                        for (IloInt i = 0; i < infeas.getSize(); i++) {
                            if (conflict[i] == IloCplex::ConflictMember)
                                std::cout << "Proved  : c" << i << infeas[i] << std::endl;
                            if (conflict[i] == IloCplex::ConflictPossibleMember)
                                std::cout << "Possible: c" << i << infeas[i] << std::endl;
                        }
                        cplex.writeConflict(conflict_path.c_str());
                    } else {
                        ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Conflict could not be refined");
                    }
                } else {
                    ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Concert exception caught: " << e);
                }

                //TODO: find better exception
                throw PlanningReport::QPFAILED;
            }
        }
        catch (...) {
            ROS_ERROR_STREAM("[GoalOptimizer] CPLEX Unknown exception caught at iteration ");
            if (not param.log_solver) {
                cplex.exportModel(QPmodel_path.c_str());
            }

            //TODO: find better exception
            throw PlanningReport::QPFAILED;
        }

        return goal;
    }

    /**
     * ===================================================================================
     * [THEORY: QP Model Construction for Subgoal Optimization]
     * ===================================================================================
     *
     * Builds the Linear Program (LP) for subgoal optimization.
     *
     * DECISION VARIABLE:
     *   t ∈ [0, 1]: Interpolation parameter on line from next_waypoint to current_goal
     *   g(t) = (current_goal - next_waypoint) * t + next_waypoint
     *
     * OBJECTIVE (minimize distance to next_waypoint):
     *   minimize t
     *   (smaller t = closer to next_waypoint = more progress toward MAPF target)
     *
     * CONSTRAINTS:
     *
     * 1. SFC (Static Flight Corridor):
     *    For each face of the bounding box:
     *      n_i · (g(t) - p_i) - d_i >= 0
     *    Ensures subgoal stays within obstacle-free region.
     *
     * 2. LSC (Linear Safe Corridor) for neighboring agents:
     *    For each agent j at final segment M-1:
     *      n_ij · (g(t) - p_j) - d_ij >= 0
     *    Ensures subgoal maintains separation from other agents.
     *
     * Note: Dynamic obstacle constraints are NOT included here because
     * the subgoal is for the FINAL position, and dynamic obstacles are
     * handled by the trajectory optimization along the path.
     *
     * [QUADRUPED ADAPTATION NOTE]:
     * For 2D navigation:
     * - SFC constraints become 4 half-planes (rectangle boundaries)
     * - LSC constraints become 2D half-planes (agent separation lines)
     * ===================================================================================
     */
    void GoalOptimizer::populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                                      const Agent &agent, const CollisionConstraints &constraints,
                                      const point3d &current_goal_point,
                                      const point3d &next_waypoint) {
        size_t N_obs = constraints.getObsSize();

        IloEnv env = model.getEnv();

        /**
         * DECISION VARIABLE: t ∈ [0, 1]
         * t = 0: g = next_waypoint (best case, fully reached target)
         * t = 1: g = current_goal (worst case, no progress)
         */
        std::string name = "t";
        x.add(IloNumVar(env, 0, 1 + SP_EPSILON_FLOAT));
        x[0].setName(name.c_str());

        /**
         * OBJECTIVE: minimize t (maximize progress toward next_waypoint)
         */
        IloNumExpr cost(env);
        cost += x[0];
        model.add(IloMinimize(env, cost));

        /**
         * CONSTRAINT 1: SFC (Static Flight Corridor)
         * Ensures subgoal g(t) stays within the static obstacle-free region.
         * SFC is converted to a set of half-plane constraints (LSC form).
         */
        if (param.world_use_octomap) {
            std::vector<LSC> lscs = constraints.getSFC(param.M - 1).convertToLSCs(param.world_dimension);
            for (const auto &lsc: lscs) {
                IloNumExpr expr(env);

                /** Half-plane constraint: n · (g(t) - p) - d >= 0 */
                for (int k = 0; k < param.world_dimension; k++) {
                    expr += lsc.normal_vector(k) *
                            ((current_goal_point(k) - next_waypoint(k)) * x[0] + next_waypoint(k) -
                             lsc.obs_control_point(k));
                }
                expr += -lsc.d;

                c.add(expr >= 0);
                expr.end();
            }
        }

        /**
         * CONSTRAINT 2: LSC for neighboring agents (Inter-agent separation)
         * Ensures subgoal maintains safe distance from other agents.
         * Only applied to non-dynamic obstacles (cooperative agents).
         */
        for (size_t oi = 0; oi < N_obs; oi++) {
            /** Skip dynamic obstacles - handled by trajectory optimization */
            if(constraints.isDynamicObstacle(oi)){
                continue;
            }

            /** Get LSC at final segment (M-1), final control point (n) */
            LSC lsc = constraints.getLSC(oi, param.M - 1, param.n);
            if (lsc.normal_vector.norm() < SP_EPSILON_FLOAT) {
                continue;
            }

            IloNumExpr expr(env);
            /** Half-plane constraint: n · (g(t) - p_obs) - d >= 0 */
            for (int k = 0; k < param.world_dimension; k++) {
                expr += lsc.normal_vector(k) * ((current_goal_point(k) - next_waypoint(k)) * x[0] + next_waypoint(k) -
                                                lsc.obs_control_point(k));
            }
            expr += -lsc.d;

            c.add(expr >= 0);
            expr.end();
        }

        model.add(c);
    }
}