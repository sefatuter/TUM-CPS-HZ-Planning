// HZFaceSampler.h: Guided state sampler for InformedHZ.
// Focuses samples on shared faces along a start→goal cell path from the
// HybridZonotope convex decomposition, with fallback to uniform sampling.


#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_INFORMEDHZ_FACE_SAMPLER_H
#define OMPL_GEOMETRIC_PLANNERS_RRT_INFORMEDHZ_FACE_SAMPLER_H

#include "ompl/base/StateSampler.h"
#include "ompl/geometric/planners/rrt/informedhz/HybridZonotope.h"
#include <vector>
#include <memory>

namespace ompl { namespace geometric { namespace informedhz {

class HZFaceSampler : public base::StateSampler
{
public:
    /** \brief Construct with SpaceInformation and a HybridZonotope handle.
     *  Keeps a default sampler for delegation and precomputes a cell path.
     */
    HZFaceSampler(const base::SpaceInformation *si, std::shared_ptr<HybridZonotope> hz);


    void sampleUniform(base::State *state) override;

    void sampleUniformNear(base::State *state, const base::State *near, double distance) override;

    void sampleGaussian(base::State *state, const base::State *mean, double stdDev) override;

protected:
    /** \brief BFS to compute the shortest list of adjacent cell indices from start to goal. */
    void computeCellPath();

    /** \brief HybridZonotope holding cells, adjacency, and shared faces. */
    std::shared_ptr<HybridZonotope> hz_;

    /** \brief Sequence of cell indices from start cell to goal cell. */
    std::vector<int> cellPath_;

    /** \brief Random number generator. */
    ompl::RNG rng_;

    base::StateSamplerPtr defaultSampler_;
};

}}} // namespace ompl::geometric::informedhz

#endif