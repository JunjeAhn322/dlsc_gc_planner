/**
 * ===================================================================================
 * [THEORY: PIBT - Priority Inheritance with Backtracking]
 * ===================================================================================
 *
 * Implementation based on:
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (IJCAI 2019).
 * "Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding"
 *
 * ALGORITHM OVERVIEW:
 * PIBT is a decentralized, real-time MAPF algorithm that:
 * 1. Assigns priorities to agents dynamically
 * 2. Uses PRIORITY INHERITANCE to resolve conflicts
 * 3. Uses BACKTRACKING when priority inheritance fails
 *
 * KEY CONCEPT - DEADLOCK RESOLUTION (Park et al. 2023/2025):
 * In the DLSC-GC framework, PIBT generates discrete waypoints that guide the
 * continuous trajectory optimization. This prevents deadlocks that can occur
 * when multiple agents have conflicting goals.
 *
 * PRIORITY ORDERING:
 * Agents are sorted by:
 * 1. obs_d: Distance to closest dynamic obstacle (higher priority if closer)
 * 2. elapsed: Time spent at goal (lower elapsed = higher priority)
 * 3. init_d: Initial distance to goal (shorter = higher priority)
 * 4. tie_breaker: Random value to break ties
 *
 * ALGORITHM STEPS:
 * 1. Sort agents by priority
 * 2. For highest priority agent, choose next node toward goal
 * 3. If node occupied by lower priority agent:
 *    - PRIORITY INHERITANCE: force lower priority agent to move first
 *    - If fails: BACKTRACK and choose different node
 * 4. If no valid node: stay in place
 *
 * [QUADRUPED ADAPTATION NOTE]:
 * For 2D navigation, the grid is a 2D lattice. Priority can be adjusted based on:
 * - Distance to dynamic obstacles (pedestrians, vehicles)
 * - Robot's payload or mission criticality
 * - Battery level or operational constraints
 * ===================================================================================
 */

#pragma once
#include "solver.hpp"

namespace MAPF {
    class PIBT : public Solver {
    public:
        static const std::string SOLVER_NAME;

    private:
        /**
         * PIBT Agent structure containing state and priority information.
         */
        struct Agent {
            int id;              ///< Unique agent identifier
            Node *v_now;         ///< Current grid node location
            Node *v_next;        ///< Planned next grid node (nullptr if not yet decided)
            Node *g;             ///< Goal grid node
            Node *o;             ///< Closest dynamic obstacle grid node (for DOI handling)
            int elapsed;         ///< Timesteps spent at goal (for priority adjustment)
            int init_d;          ///< Initial distance to goal (for tie-breaking)
            float tie_breaker;   ///< Random tie-breaker for deterministic ordering
            float obs_d;         ///< Distance to closest dynamic obstacle
        };
        using Agents = std::vector<Agent *>;

        /**
         * Reservation tables for collision avoidance.
         * occupied_now: which agent is at each node at current timestep
         * occupied_next: which agent will be at each node at next timestep
         */
        Agents occupied_now;
        Agents occupied_next;

        bool disable_dist_init = false;

        /**
         * Core PIBT function with Priority Inheritance and Backtracking.
         * @param ai Agent to plan for
         * @return true if successfully planned, false if stuck
         */
        bool funcPIBT(Agent *ai);

        /** Plan one step toward goal, considering reservations */
        Node *planOneStep(Agent *a);

        /** Choose best node from candidates based on distance heuristics */
        Node *chooseNode(Agent *a);

        /** Main PIBT loop */
        void run();

    public:
        PIBT(Problem *_P);

        ~PIBT() {}

        void setParams(int argc, char *argv[]);

        static void printHelp();

        /** Compute Euclidean distance from agent's obstacle to a node */
        static float obsDist(Agent *a, Node* s);
    };
}