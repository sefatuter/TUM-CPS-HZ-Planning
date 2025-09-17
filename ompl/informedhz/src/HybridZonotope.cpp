// HybridZonotope.cpp
// Implementation of the geometric free-space construction and convex decomposition using CGAL.
// Builds a convex-cell decomposition of the (bounds \ obstacles) region and exposes queries.
#include "ompl/geometric/planners/rrt/informedhz/HybridZonotope.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Console.h"

// CGAL headers
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/boost/graph/helpers.h>

#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace ompl { namespace geometric { namespace informedhz {

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Polyhedron_3 = CGAL::Polyhedron_3<Kernel>;
using Nef_3 = CGAL::Nef_polyhedron_3<Kernel>;
namespace PMP = CGAL::Polygon_mesh_processing;

struct HybridZonotope::CGALImpl {
    std::unique_ptr<Nef_3> freeSpaceNef;                         
    std::vector<Nef_3::Volume_const_iterator> cgalVolumeIterators;
};

// --- Lifecycle -------------------------------------------------------------------------------

/** Construct with OMPL SpaceInformation; allocate CGAL impl holder. */
HybridZonotope::HybridZonotope(const base::SpaceInformationPtr &si)
    : si_(si), impl_(std::make_unique<CGALImpl>())
{
}

/** Default destructor (PIMPL deallocates automatically). */
HybridZonotope::~HybridZonotope() = default;

// --- Internal state resets -------------------------------------------------------------------

/** Clear all data derived from bounds/obstacles (cells, adjacency, shared faces, cached Nef). */
void HybridZonotope::clearComputedData_()
{
    cells_.clear();
    adjacency_.clear();
    sharedFaceVertices_.clear();
    startCell_ = goalCell_ = -1;
    impl_->freeSpaceNef.reset();
    impl_->cgalVolumeIterators.clear();
}

// --- Environment setup (bounds, start/goal, obstacles) ---------------------------------------

void HybridZonotope::setBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
    dim_ = 3;
    bounds_ = {xmin, xmax, ymin, ymax, zmin, zmax};
    clearComputedData_();
}

/** Set 2D bounds [xmin,xmax]×[ymin,ymax]; creates a very thin slab in z; invalidates data. */
void HybridZonotope::setBounds(double xmin, double xmax, double ymin, double ymax)
{
    dim_ = 2;
    bounds_ = {xmin, xmax, ymin, ymax, -0.0005, 0.0005};
    clearComputedData_();
}

void HybridZonotope::setStart(const std::vector<double> &s)
{
    if (s.size() < 2) return;
    start_[0] = s[0];
    start_[1] = s[1];
    start_[2] = (s.size() >= 3) ? s[2] : 0.0;
}

void HybridZonotope::setGoal(const std::vector<double> &g)
{
    if (g.size() < 2) return;
    goal_[0] = g[0];
    goal_[1] = g[1];
    goal_[2] = (g.size() >= 3) ? g[2] : 0.0;
}

void HybridZonotope::clearObstacles()
{
    obstacles_.clear();
    clearComputedData_();
}

void HybridZonotope::addMeshObstacle(const ObstacleMesh &obs)
{
    obstacles_.push_back(obs);
    clearComputedData_();
}

void HybridZonotope::addRectangleObstacle2D(double x, double y, double w, double h)
{
    std::vector<std::array<double, 2>> poly = {
        {{x, y}}, {{x + w, y}}, {{x + w, y + h}}, {{x, y + h}}
    };
    addPolygonObstacle2D(poly);
}

void HybridZonotope::addPolygonObstacle2D(const std::vector<std::array<double, 2>> &poly2d)
{
    if (poly2d.size() < 3) return;

    const double zmin = bounds_[4];
    const double zmax = bounds_[5];
    
    ObstacleMesh prism;
    prism.vertices.reserve(poly2d.size() * 2);
    
    // bottom ring (zmin)
    for (const auto &p : poly2d) prism.vertices.push_back({p[0], p[1], zmin});
    // top ring (zmax)
    for (const auto &p : poly2d) prism.vertices.push_back({p[0], p[1], zmax});

    const int N = static_cast<int>(poly2d.size());

    // faces: bottom (reverse order for outward orientation), top, and side quads as two triangles
    std::vector<int> bottom_face, top_face;
    for (int i = 0; i < N; ++i) {
        bottom_face.push_back(N - 1 - i); // reverse bottom
        top_face.push_back(i + N);        // top in forward order
    }
    prism.faces.push_back(bottom_face);
    prism.faces.push_back(top_face);
    for (int i = 0; i < N; ++i) {
        int j = (i + 1) % N;
        int v0 = i, v1 = j;
        int v2 = i + N, v3 = j + N;
        prism.faces.push_back({v0, v1, v3});   // first tri
        prism.faces.push_back({v0, v3, v2});   // second tri
    }
    addMeshObstacle(prism);
}

// --- Main build: construct (box \ obstacles), decompose into convex cells, and index them -----

bool HybridZonotope::performConvexDecomposition()
{
    clearComputedData_();

    // 1) Build world box as a Polyhedron
    Polyhedron_3 box_poly;
    CGAL::make_hexahedron(
        Point_3(bounds_[0], bounds_[2], bounds_[4]), Point_3(bounds_[1], bounds_[2], bounds_[4]),
        Point_3(bounds_[1], bounds_[3], bounds_[4]), Point_3(bounds_[0], bounds_[3], bounds_[4]),
        Point_3(bounds_[0], bounds_[2], bounds_[5]), Point_3(bounds_[1], bounds_[2], bounds_[5]),
        Point_3(bounds_[1], bounds_[3], bounds_[5]), Point_3(bounds_[0], bounds_[3], bounds_[5]),
        box_poly);
    
    // 2) Subtract obstacles: freeSpace = box \ union(obstacles)
    Nef_3 freeSpace(box_poly);
    if (!obstacles_.empty()) {
        Nef_3 all_obstacles_nef(Nef_3::EMPTY);

        for (const auto &obs : obstacles_) {
            // Convert obstacle soup (vertices + face indices) into a Polyhedron
            std::vector<Point_3> cgal_points; cgal_points.reserve(obs.vertices.size());
            for (const auto &v : obs.vertices) cgal_points.emplace_back(v[0], v[1], v[2]);

            std::vector<std::vector<std::size_t>> cgal_faces; cgal_faces.reserve(obs.faces.size());
            for (const auto &f : obs.faces) cgal_faces.emplace_back(f.begin(), f.end());

            Polyhedron_3 obs_poly;
            PMP::orient_polygon_soup(cgal_points, cgal_faces);
            PMP::polygon_soup_to_polygon_mesh(cgal_points, cgal_faces, obs_poly);

            // Ensure closed & outward orientation before turning into a Nef
            if (!obs_poly.is_empty() && CGAL::is_closed(obs_poly) && !PMP::is_outward_oriented(obs_poly))
                PMP::reverse_face_orientations(obs_poly);

            all_obstacles_nef += Nef_3(obs_poly);
        }

        freeSpace -= all_obstacles_nef;
    }

    // 3) Convex decomposition on the free-space Nef
    try {
        CGAL::convex_decomposition_3(freeSpace);
    } catch (const std::exception& e) {
        OMPL_ERROR("CGAL convex_decomposition_3 failed: %s.", e.what());
        return false;
    }

    // Keep the decomposed Nef alive in impl_
    impl_->freeSpaceNef = std::make_unique<Nef_3>(freeSpace);

    // 4) Extract convex cells (interior volumes are 'marked')
    // Use a map keyed by Volume_const_handle (hash/ordering is defined) → robust lookups later.
    std::unordered_map<Nef_3::Volume_const_iterator, int> cgal_vol_to_idx;

    // Skip the first volume (the unbounded exterior); iterate interior volumes
    for (auto vol_it = ++impl_->freeSpaceNef->volumes_begin(); vol_it != impl_->freeSpaceNef->volumes_end(); ++vol_it) {
        if (!vol_it->mark()) continue; // only interior (marked) volumes are convex cells

        // Convert the (inner) shell of this volume to a Polyhedron
        Polyhedron_3 cell_poly;
        impl_->freeSpaceNef->convert_inner_shell_to_polyhedron(vol_it->shells_begin(), cell_poly);
        if (cell_poly.is_empty()) continue;

        // Copy vertices/faces into our compact ConvexCell
        ConvexCell cell;
        std::unordered_map<Polyhedron_3::Vertex_const_handle, int> vh_to_id;
        int vertex_id = 0;

        for (auto v = cell_poly.vertices_begin(); v != cell_poly.vertices_end(); ++v) {
            const auto &p = v->point();
            cell.vertices.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
            vh_to_id[v] = vertex_id++;
        }

        for (auto f = cell_poly.facets_begin(); f != cell_poly.facets_end(); ++f) {
            std::vector<int> face_indices;
            auto h = f->facet_begin();
            do { face_indices.push_back(vh_to_id.at(h->vertex())); } while (++h != f->facet_begin());
            cell.faces.push_back(face_indices);
        }
        cells_.push_back(cell);
        impl_->cgalVolumeIterators.push_back(vol_it);
        cgal_vol_to_idx[vol_it] = cells_.size() - 1;
    }
    if (cells_.empty()) {
        OMPL_WARN("Convex decomposition resulted in no valid cells.");
        return false;
    }

    // 5) Build cell adjacency from Nef halffacets (shared boundary between two interior volumes)   
    adjacency_.assign(cells_.size(), std::vector<bool>(cells_.size(), false));
    
    Nef_3::Halffacet_const_iterator hf_it;
    CGAL_forall_halffacets(hf_it, *impl_->freeSpaceNef)
    {
        Nef_3::Volume_const_handle vol1 = hf_it->incident_volume();
        Nef_3::Volume_const_handle vol2 = hf_it->twin()->incident_volume();

        auto it1 = cgal_vol_to_idx.find(vol1);
        auto it2 = cgal_vol_to_idx.find(vol2);
        if (it1 != cgal_vol_to_idx.end() && it2 != cgal_vol_to_idx.end())
        {
            int i = it1->second;
            int j = it2->second;
            adjacency_[i][j] = true;
            adjacency_[j][i] = true;
        }
    }

    // 6) Find which convex cells contain start and goal points
    auto locate_point = [&](const std::array<double, 3> &p) -> int {
        if (!impl_->freeSpaceNef) return -1;
        auto obj = impl_->freeSpaceNef->locate(Point_3(p[0], p[1], p[2]));
        Nef_3::Volume_const_handle vh;
        if (CGAL::assign(vh, obj) && cgal_vol_to_idx.count(vh)) {
            return cgal_vol_to_idx.at(vh);
        }
        return -1;
    };

    startCell_ = locate_point(start_);
    goalCell_ = locate_point(goal_);
    
    if (startCell_ == -1) OMPL_WARN("Start state is outside the valid decomposed free space.");
    if (goalCell_ == -1) OMPL_WARN("Goal state is outside the valid decomposed free space.");
    
    OMPL_INFORM("HybridZonotope: Decomposition complete. Found %zu convex cells. Start cell: %d, Goal cell: %d",
                cells_.size(), startCell_, goalCell_);
    
                return true;
}

// --- Queries ----------------------------------------------------------------------------------

/** Point-in-free-space query using the decomposed Nef:
 *  - Checks bounds first, then uses Nef::locate to see if point lies in a marked (interior) volume.
 */
bool HybridZonotope::isStateFree(const ompl::base::State *s) const
{
    const auto *rv_space = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();
    const auto *rv_state = s->as<ompl::base::RealVectorStateSpace::StateType>();

    if (!rv_space || !rv_state) return false;
    
    double x = rv_state->values[0];
    double y = rv_state->values[1];
    double z = (dim_ == 3 && rv_space->getDimension() >= 3) ? rv_state->values[2] : 0.0;
    
    // Quick AABB bounds check
    if (x < bounds_[0] || x > bounds_[1] || y < bounds_[2] || y > bounds_[3] || z < bounds_[4] || z > bounds_[5])
        return false;

    // If we don't have a Nef built, treat as invalid (caller may install fallback checker)
    if (!impl_->freeSpaceNef) {
        return false;
    }

    auto obj = impl_->freeSpaceNef->locate(Point_3(x, y, z));
    Nef_3::Volume_const_handle vh;
    if (CGAL::assign(vh, obj)) {
        // Marked = interior of free space  
        return vh->mark();
    }
    // If not a volume (on boundary), treat as valid by default.
    return true;
}

}}} // namespace ompl::geometric::informedhz