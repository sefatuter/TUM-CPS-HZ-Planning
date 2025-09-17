#include "ompl/geometric/planners/rrt/InformedHZ.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Console.h"

namespace ompl { namespace geometric {

bool isStateInsideObstacles(const informedhz::HybridZonotope* hz, const base::State* s)
{
    const auto* rstate = s->as<base::RealVectorStateSpace::StateType>();
    double x = rstate->values[0];
    double y = rstate->values[1];

    const auto& obstacles = hz->getObstacles();
    const auto& bounds = hz->getBounds();

    for (const auto& obs_mesh : obstacles)
    {
        double min_x = 1e10, max_x = -1e10, min_y = 1e10, max_y = -1e10;
        bool has_base_verts = false;
        for(const auto& v : obs_mesh.vertices) {
            if (std::abs(v[2] - bounds[4]) < 1e-6) {
                min_x = std::min(min_x, v[0]);
                max_x = std::max(max_x, v[0]);
                min_y = std::min(min_y, v[1]);
                max_y = std::max(max_y, v[1]);
                has_base_verts = true;
            }
        }
        if (has_base_verts && x >= min_x && x <= max_x && y >= min_y && y <= max_y)
            return true;
    }
    return false;
}

// Planner constructor.
InformedHZ::InformedHZ(const base::SpaceInformationPtr &si) : RRTstar(si)
{
    setName("InformedHZ");
}

// Configure a 2D problem with rectangular obstacles.
void InformedHZ::useRectangles2D(double xmin, double xmax, double ymin, double ymax,
                                 const std::vector<std::array<double, 4>> &rects,
                                 const std::array<double, 2> &start,
                                 const std::array<double, 2> &goal)
{
    hz_ = std::make_shared<informedhz::HybridZonotope>(si_);
    hz_->setBounds(xmin, xmax, ymin, ymax);
    hz_->setStart({start[0], start[1]});
    hz_->setGoal({goal[0], goal[1]});
    for (const auto &r : rects)
        hz_->addRectangleObstacle2D(r[0], r[1], r[2], r[3]);
    isSetup_ = false; 
}

// Configure a 2D problem with polygonal obstacles.
void InformedHZ::usePolygons2D(double xmin, double xmax, double ymin, double ymax,
                               const std::vector<std::vector<std::array<double, 2>>> &polys,
                               const std::array<double, 2> &start,
                               const std::array<double, 2> &goal)
{
    hz_ = std::make_shared<informedhz::HybridZonotope>(si_);
    hz_->setBounds(xmin, xmax, ymin, ymax);
    hz_->setStart({start[0], start[1]});
    hz_->setGoal({goal[0], goal[1]});
    for (const auto &poly : polys)
        hz_->addPolygonObstacle2D(poly);
    isSetup_ = false; // Mark that setup needs to be re-run
}

// One-time preparation of HybridZonotope and the validity checker.
void InformedHZ::ensureHZIsReady_()
{
    if (isSetup_) return;
    OMPL_WARN("InformedHZ: TESTT ENSURE HZ ");

    if (!hz_) { /* Logic to create default HZ */ 
        OMPL_WARN("Checking hz_ pointer. Address: %p", hz_.get());
        OMPL_WARN("InformedHZ: TESTT DEFAULT HZ CREATED");
    }

    bool success = (hz_ && hz_->performConvexDecomposition());

    if (success)
    {
        OMPL_INFORM("%s: Convex decomposition successful. Using fast checker.", getName().c_str());
        wireValidityChecker_();
    }
    else
    {
        OMPL_ERROR("%s: Convex decomposition failed. Using slower, fallback validity checker.", getName().c_str());
        wireFallbackValidityChecker_();
    }
    isSetup_ = true;
}

// Install the fast validity checker (requires successful convex decomposition).
// Uses hz_->isStateFree(s) for exact, efficient collision queries.
void InformedHZ::wireValidityChecker_()
{
    if (!hz_) return;
    si_->setStateValidityChecker([hz = hz_](const base::State *s) { return hz->isStateFree(s); });
}

// Install a conservative, slower fallback checker.
// Marks a state invalid if it falls inside any obstacle’s base-plane AABB.
void InformedHZ::wireFallbackValidityChecker_()
{
    if (!hz_) return;
    // To call the helper, we pass `hz.get()` to get the raw pointer.
    si_->setStateValidityChecker([hz = hz_](const base::State *s) { return !isStateInsideObstacles(hz.get(), s); });
}

// Standard OMPL setup.
void InformedHZ::setup()
{
    OMPL_WARN("InformedHZ: TESTT SETUP DEBUG");
    RRTstar::setup();
    if (!pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to path length optimization.", getName().c_str());
        pdef_->setOptimizationObjective(std::make_shared<base::PathLengthOptimizationObjective>(si_));
    }
    ensureHZIsReady_();
}

// Reset planner and mark setup as dirty.
void InformedHZ::clear()
{
    OMPL_WARN("InformedHZ: TESTT CLEAR");
    RRTstar::clear();
    isSetup_ = false; // Allow setup to run again on the next solve call
}

base::PlannerStatus InformedHZ::solve(const base::PlannerTerminationCondition &ptc)
{
    return RRTstar::solve(ptc);
}

}} // namespace ompl::geometric