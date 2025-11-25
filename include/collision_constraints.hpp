#ifndef DLSC_GC_PLANNER_COLLISION_CONSTRAINTS_HPP
#define DLSC_GC_PLANNER_COLLISION_CONSTRAINTS_HPP

#include <vector>
#include <octomap/octomap_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>
#include <trajectory.hpp>
#include <convhull_3d/convhull_3d.h>

namespace MATP {
    /**
     * ===================================================================================
     * [THEORY: Linear Safe Corridor (LSC) - Park et al. 2023/2025, Section 4]
     * ===================================================================================
     *
     * LSC defines a half-space constraint that separates the agent from an obstacle.
     *
     * MATHEMATICAL DEFINITION:
     *   LSC = {c ∈ R³ | (c - c_obs) · n - d > 0}
     *
     * where:
     *   c       = trajectory control point (decision variable)
     *   c_obs   = obstacle reference point (control point on obstacle trajectory)
     *   n       = separation hyperplane normal vector (points toward agent)
     *   d       = safety margin (collision distance)
     *
     * GEOMETRIC INTERPRETATION:
     * The LSC defines a half-plane that the agent must stay on. The normal vector
     * points away from the obstacle toward the agent's safe region.
     *
     * TYPES OF LSC IN THIS IMPLEMENTATION:
     * 1. LSC for agents (DLSC-GC): GJK-based normal vector for convex hull separation
     * 2. RSFC for dynamic obstacles: Tangent plane to inflated collision sphere
     * 3. BVC (Buffered Voronoi Cell): Voronoi-based separation (simpler variant)
     *
     * [QUADRUPED ADAPTATION NOTE]:
     * For 2D navigation, LSC becomes a half-plane in 2D:
     *   {(x,y) | (x-x_obs)*n_x + (y-y_obs)*n_y - d > 0}
     * ===================================================================================
     */
    class LSC {
    public:
        LSC() = default;

        LSC(const point3d &obs_control_point,
            const point3d &normal_vector,
            double d);

        [[nodiscard]] visualization_msgs::Marker convertToMarker(double agent_radius,
                                                                 const std::string &world_frame_id) const;

        /** Check if a point satisfies this LSC constraint */
        bool isPointInLSC(const point3d &point) const;

        point3d obs_control_point;  ///< Reference point on obstacle trajectory c_obs
        point3d normal_vector;       ///< Separation hyperplane normal n (unit vector toward agent)
        double d = 0;                ///< Safety margin (collision distance r_agent + r_obs)
    };

    typedef std::vector<LSC> LSCs;

    /**
     * Container for all dynamic obstacle constraints across the planning horizon.
     * Stores LSC[obstacle_idx][segment_idx][control_point_idx].
     */
    class DynamicConstraints {
    public:
        DynamicConstraints() = default;
        int N_obs = 0;                    ///< Number of obstacles (agents + dynamic obs)
        ObstacleTypes types;              ///< Type of each obstacle (AGENT, DYN_REAL, etc.)
        point3ds obs_positions;           ///< Current positions of obstacles
        std::vector<std::vector<std::vector<LSC>>> constraints; ///< [obs_idx][segment_idx][ctrl_pt_idx]

        void initialize(int N_obs_, int M, int n);
        LSC getLSC(int oi, int m, int i);
    };

    /**
     * ===================================================================================
     * [THEORY: Static Flight Corridor (SFC) - Park et al. 2023/2025]
     * ===================================================================================
     *
     * SFC defines an axis-aligned bounding box (AABB) that is free of static obstacles.
     * The agent's trajectory must stay within the SFC to avoid static collisions.
     *
     * MATHEMATICAL DEFINITION:
     *   SFC = {c ∈ R³ | box_min < c < box_max}
     *
     * Equivalent to 6 half-plane constraints:
     *   c_x > box_min_x,  c_x < box_max_x
     *   c_y > box_min_y,  c_y < box_max_y
     *   c_z > box_min_z,  c_z < box_max_z
     *
     * SFC CONSTRUCTION (Axis Search Method):
     * 1. Start with a small AABB containing the agent's current position
     * 2. Expand in each axis direction until hitting a static obstacle
     * 3. The resulting box is the largest obstacle-free region
     *
     * [QUADRUPED ADAPTATION NOTE]:
     * For 2D navigation, SFC becomes a rectangle:
     *   {(x,y) | x_min < x < x_max, y_min < y < y_max}
     * The z dimension can be ignored or fixed to terrain height.
     * ===================================================================================
     */
    class Box {
    public:
        point3d box_min;
        point3d box_max;

        Box() = default;

        Box(const point3d &box_min, const point3d &box_max);

        [[nodiscard]] LSCs convertToLSCs(int dim) const;

        [[nodiscard]] visualization_msgs::Marker convertToMarker(double agent_radius,
                                                                 const std::string &world_frame_id) const;

        [[nodiscard]] bool isPointInBox(const point3d &point) const;

        [[nodiscard]] bool isPointInBoxStrictly(const point3d &point) const;

        [[nodiscard]] bool isLineInBox(const Line &line) const;

        [[nodiscard]] bool isSegmentInBox(const Segment<point3d> &segment) const;

        [[nodiscard]] bool isSegmentInBox(const Segment<point3d> &segment, double margin) const;

        [[nodiscard]] bool isInOtherBox(const point3d &world_min, const point3d &world_max, double margin) const;

        [[nodiscard]] bool isSuperSetOfConvexHull(const point3ds &convex_hull) const;

        [[nodiscard]] bool intersectWith(const Box &other_box) const;

        [[nodiscard]] bool tangentWith(const Box &other_box) const;

        [[nodiscard]] bool include(const Box &other_box) const;

        [[nodiscard]] Box unify(const Box &other_box) const;

        [[nodiscard]] Box intersection(const Box &other_box) const;

        [[nodiscard]] point3d closestPoint(const point3d &point) const;

        [[nodiscard]] double distanceToPoint(const point3d &point) const;

        [[nodiscard]] double distanceToInnerPoint(const point3d &point) const;

        [[nodiscard]] double raycastFromInnerPoint(const point3d &point, const point3d &direction) const;

        [[nodiscard]] double raycastFromInnerPoint(const point3d &point, const point3d &direction,
                                                   point3d &surface_direction) const;

        [[nodiscard]] point3ds getVertices(int dim, double z_2d) const;

        [[nodiscard]] lines_t getEdges() const;

        [[nodiscard]] point3ds getSurfacePoints(double resolution) const;

        bool operator==(Box &other_sfc) const;

        bool operator!=(Box &other_sfc) const;
    };
    
    typedef std::vector<Box> SFCs; // [segment_idx]
    typedef std::vector<Box> Boxes;

    class CollisionConstraints {
    public:
        CollisionConstraints(const Param &param, const Mission &mission, double radius, double max_vel);

        void initializeSFC(const point3d &agent_position);

        void initializeLSC(const Obstacles &obstacle);

        void constructSFCFromPoint(const point3d &point, const point3d &goal_point);

        void constructSFCFromConvexHull(const point3ds &convex_hull,
                                        const point3d &next_waypoint);

        void constructSFCFromInitialTraj(const traj_t &initial_traj,
                                         const point3d &current_goal_point,
                                         const point3d &next_waypoint);

        void constructCommunicationRange(const point3d &next_waypoint);

//        std::vector<SFCs> findSFCsCandidates(const std::vector<SFCs> &sfcs_valid);
//
//        std::vector<SFCs> findSFCsCandidates(const std::vector<SFCs> &sfcs_valid,
//                                             const Box &sfc_prev,
//                                             int m);

        [[nodiscard]] bool isDynamicObstacle(int oi) const;

        [[nodiscard]] bool isPointInFeasibleRegion(const point3d &point, int m, int i) const;

        // Getter
        [[nodiscard]] LSC getLSC(int oi, int m, int i) const;

        [[nodiscard]] Box getSFC(int m) const;

        [[nodiscard]] size_t getObsSize() const;

        [[nodiscard]] point3d getObsPosition(int oi) const;

        // Setter
        void setDistmap(std::shared_ptr<DynamicEDTOctomap> distmap_ptr);

        void setOctomap(std::shared_ptr<octomap::OcTree> octree_ptr);

        void setLSC(int oi, int m, int i, const LSC& lsc);

        void setSFC(int m, const Box &sfc);

        // Converter
        [[nodiscard]] visualization_msgs::MarkerArray convertLSCsToMarkerArrayMsg(
                const Obstacles &obstacles,
                const Colors &colors) const;

        [[nodiscard]] visualization_msgs::MarkerArray convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA &color) const;

        [[nodiscard]] visualization_msgs::MarkerArray
        feasibleRegionToMarkerArrayMsg(int agent_id, const std_msgs::ColorRGBA &color) const;

    private:
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;
        std::shared_ptr<octomap::OcTree> octree_ptr;
        Mission mission;
        Param param;

        double agent_radius, agent_max_vel;

        DynamicConstraints lscs; // Safe corridor to avoid agents and dynamic obstacles,
        SFCs sfcs; // Safe corridor to avoid static obstacles
        Box communication_range;

        bool expandSFCFromPoint(const point3d &point, const point3d &goal_point,
                                const Box &sfc_prev, Box &sfc_expand);

        bool expandSFCFromConvexHull(const point3ds &convex_hull, Box &sfc_expand);

        bool expandSFCFromConvexHull(const point3ds &convex_hull, const Box &sfc_prev, Box &sfc_expand);

        bool isObstacleInSFC(const Box &sfc_initial, double margin);

        bool isSFCInBoundary(const Box &sfc, double margin);

//        Boxes findSurfaceBoxes(const Box &sfc);

//        Box unifySurfaceBoxes(const Boxes &surface_boxes);

//        Box shrinkUnionBox(const Box& union_box, const Box &sfc_initial, const Boxes &surface_boxes);

//        bool expandSFCUniformly(const Box &sfc_initial, double margin, Box &sfc_expand);

        bool expandSFCIncrementally(const Box &sfc_initial, double margin, Box &sfc_expand);

//        bool expandSFCLInfinity(const Box &sfc_initial, double margin, Box &sfc_expand);

        bool expandSFC(const Box &sfc_initial, const point3d &goal_point, double margin, Box &sfc_expand);

        [[nodiscard]] point3ds findFeasibleVertices(int m, int control_point_idx) const;

        static visualization_msgs::Marker convexHullToMarkerMsg(const point3ds &convex_hull,
                                                                const std_msgs::ColorRGBA &color,
                                                                const std::string &world_frame_id);

        static std::vector<int> setAxisCand(const Box &box, const octomap::point3d &goal_point);
    };
}


#endif //DLSC_GC_PLANNER_COLLISION_CONSTRAINTS_HPP
