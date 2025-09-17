// HZFaceSampler.cpp: Implementation of the guided state sampler.

#include "ompl/geometric/planners/rrt/informedhz/HZFaceSampler.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <queue>
#include <algorithm>

namespace ompl { namespace geometric { namespace informedhz {


HZFaceSampler::HZFaceSampler(const base::SpaceInformation *si, std::shared_ptr<HybridZonotope> hz)
    : base::StateSampler(si->getStateSpace().get()), hz_(std::move(hz))
{
    defaultSampler_ = si->allocStateSampler();
    computeCellPath();
}

// Use Breadth-First Search (BFS) to find the shortest sequence of adjacent cells.

void HZFaceSampler::computeCellPath()
{
    int start = hz_->getStartCell();
    int goal = hz_->getGoalCell();

    if (start < 0 || goal < 0) {
        OMPL_WARN("HZFaceSampler: Cannot compute cell path because start or goal cell is invalid.");
        return;
    }
    if (start == goal) {
        cellPath_ = {start};
        OMPL_INFORM("HZFaceSampler: Start and goal are in the same cell.");
        return;
    }

    std::queue<std::vector<int>> q;
    q.push({start});
    std::vector<bool> visited(hz_->getCells().size(), false);
    visited[start] = true;

    while (!q.empty()) {
        std::vector<int> path = q.front();
        q.pop();
        int current = path.back();

        if (current == goal) {
            cellPath_ = path;
            OMPL_INFORM("HZFaceSampler: Found cell path of length %zu.", cellPath_.size());
            return;
        }

        const auto& neighbors = hz_->getAdjacency()[current];
        for (size_t i = 0; i < neighbors.size(); ++i) {
            if (neighbors[i] && !visited[i]) {
                visited[i] = true;
                std::vector<int> new_path = path;
                new_path.push_back(i);
                q.push(new_path);
            }
        }
    }
    OMPL_WARN("HZFaceSampler: Could not find a path of cells from start to goal.");
}


void HZFaceSampler::sampleUniform(base::State *state)
{
    // No guided path available → fall back to global uniform sampling.
    if (cellPath_.empty() || cellPath_.size() < 2) {
        defaultSampler_->sampleUniform(state);
        return;
    }

    auto *rstate = state->as<base::RealVectorStateSpace::StateType>();
    
    // Choose a random adjacent pair (cellA → cellB) along the precomputed path.
    size_t path_idx = rng_.uniformInt(0, cellPath_.size() - 2);
    int cellA_idx = cellPath_[path_idx];
    int cellB_idx = cellPath_[path_idx + 1];

    if (rng_.uniform01() < 0.95) {
        const auto& face_verts = hz_->getSharedFaces().at({cellA_idx, cellB_idx});
        
        if (face_verts.size() >= 3) {
            double b1 = rng_.uniform01();
            double b2 = rng_.uniform01();
            if (b1 + b2 > 1.0) {
                b1 = 1.0 - b1;
                b2 = 1.0 - b2;
            }
            double b3 = 1.0 - b1 - b2;
            
            rstate->values[0] = b1 * face_verts[0][0] + b2 * face_verts[1][0] + b3 * face_verts[2][0];
            rstate->values[1] = b1 * face_verts[0][1] + b2 * face_verts[1][1] + b3 * face_verts[2][1];
            if(space_->getDimension() > 2)
                 rstate->values[2] = b1 * face_verts[0][2] + b2 * face_verts[1][2] + b3 * face_verts[2][2];
        } else {
            // Not enough verts to define a face → delegate.
            defaultSampler_->sampleUniform(state);
        }
    } else {
        const auto& cell = hz_->getCells()[(rng_.uniform01() < 0.5 ? cellA_idx : cellB_idx)];
        
        double min_x = 1e10, max_x = -1e10, min_y = 1e10, max_y = -1e10;
        for(const auto& v : cell.vertices) {
            min_x = std::min(min_x, v[0]);
            max_x = std::max(max_x, v[0]);
            min_y = std::min(min_y, v[1]);
            max_y = std::max(max_y, v[1]);
        }
        rstate->values[0] = rng_.uniformReal(min_x, max_x);
        rstate->values[1] = rng_.uniformReal(min_y, max_y);
    }
}

void HZFaceSampler::sampleUniformNear(base::State *state, const base::State *near, double distance)
{
    defaultSampler_->sampleUniformNear(state, near, distance);
}

void HZFaceSampler::sampleGaussian(base::State *state, const base::State *mean, double stdDev)
{
    defaultSampler_->sampleGaussian(state, mean, stdDev);
}

}}}