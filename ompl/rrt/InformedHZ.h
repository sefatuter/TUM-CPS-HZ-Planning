#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_HZ_H
#define OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_HZ_H

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/rrt/informedhz/HybridZonotope.h" // Assumes this header is in the same directory
#include <memory>
#include <vector>
#include <array>

namespace ompl { namespace geometric {

/**
 * InformedHZ planner (RRT* variant) that uses a HybridZonotope-based
*/
class InformedHZ : public RRTstar
{
public:
    // Constructor: set up planner with the given SpaceInformation.
    explicit InformedHZ(const base::SpaceInformationPtr &si);

    // Runs standard OMPL setup; installs default objective if missing and wires the validity checker.
    void setup() override;

    // Clears planner state and marks internal setup as dirty (forces re-prepare on next run).
    void clear() override;

    // Delegates solving to RRT*; assumes setup() prepared objectives and validity checking.
    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

    // --- Configuration ---

    void useRectangles2D(double xmin, double xmax, double ymin, double ymax,
                         const std::vector<std::array<double, 4>> &rects,
                         const std::array<double, 2> &start,
                         const std::array<double, 2> &goal);
    
    void usePolygons2D(double xmin, double xmax, double ymin, double ymax,
                       const std::vector<std::vector<std::array<double, 2>>> &polys,
                       const std::array<double, 2> &start,
                       const std::array<double, 2> &goal);

protected:
    // One-time preparation: run convex decomposition and wire the appropriate validity checker.
    void ensureHZIsReady_();

    void wireValidityChecker_();

    void wireFallbackValidityChecker_();

private:
    // HybridZonotope handle holding bounds, obstacles, and decomposition state.
    std::shared_ptr<informedhz::HybridZonotope> hz_;

    // Internal flag: true after ensureHZIsReady_() has successfully chosen and wired a checker.
    bool isSetup_{false};
};

}} // namespace ompl::geometric

#endif