// HybridZonotope.h
// Declaration of the core class that manages bounds, obstacles, convex decomposition,
// and queries/metadata (cells, adjacency, shared faces, start/goal cells).

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_INFORMEDHZ_HYBRID_ZONOTOPE_H
#define OMPL_GEOMETRIC_PLANNERS_RRT_INFORMEDHZ_HYBRID_ZONOTOPE_H

#include "ompl/base/SpaceInformation.h"
#include <array>
#include <vector>
#include <map>
#include <memory>
#include <utility>

namespace ompl { namespace geometric { namespace informedhz {

class HybridZonotope {
public:
    // --- Public geometry types ---------------------------------------------------------------

    /** Triangle-mesh obstacle: vertices in 3D, faces as index lists into 'vertices'. */
    struct ObstacleMesh {
        std::vector<std::array<double, 3>> vertices;
        std::vector<std::vector<int>>     faces;
    };

    /** Convex cell produced by decomposition: vertices + faces (indices) for adjacency/visualization. */
    struct ConvexCell {
        std::vector<std::array<double, 3>> vertices;
        std::vector<std::vector<int>>     faces;
    };

    /** Shared face map: optional list of shared-face vertices keyed by (cell_i, cell_j). */
    using SharedFaceMap = std::map<std::pair<int, int>, std::vector<std::array<double, 3>>>;

public:
    // --- Lifecycle ---------------------------------------------------------------------------

    /** Construct with OMPL SpaceInformation (used for state-space dimension checks). */
    explicit HybridZonotope(const ompl::base::SpaceInformationPtr& si);
    ~HybridZonotope();

    // --- Environment Setup (invalidates previous decomposition) ------------------------------

    void setBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

    void setBounds(double xmin, double xmax, double ymin, double ymax); // 2D convenience

    /** Set start position (expects at least x,y; z defaults to 0 if omitted). */
    void setStart(const std::vector<double>& s);

    /** Set goal position (expects at least x,y; z defaults to 0 if omitted). */
    void setGoal(const std::vector<double>& g);

    /** Add a full 3D mesh obstacle (triangles in 'faces' index into 'vertices'). */
    void addMeshObstacle(const ObstacleMesh& obs);

    /** Convenience: add an axis-aligned rectangle (x,y,width,height) as a thin 3D prism. */
    void addRectangleObstacle2D(double x, double y, double w, double h);
    
    /** Add an arbitrary simple polygon (list of {x,y} vertices) as a thin 3D prism. */
    void addPolygonObstacle2D(const std::vector<std::array<double, 2>>& poly2d);

    /** Remove all obstacles and invalidate derived data. */
    void clearObstacles();

    // --- Main Build Step ---------------------------------------------------------------------

    /** Build free space as a Nef (world box \ obstacles), run convex decomposition, and extract cells.
     *  Returns true on success (≥1 cell), false if decomposition failed or yielded 0 cells.
     */
    bool performConvexDecomposition();


    // --- Queries & Accessors -----------------------------------------------------------------

    /** Point-in-free-space query using the decomposed Nef (bounds check + Nef locate). */
    bool isStateFree(const ompl::base::State* s) const;

    /** All convex cells (post-decomposition). */
    const std::vector<ConvexCell>&          getCells() const { return cells_; }
    /** Symmetric adjacency matrix (cells share a boundary ⇒ true). */
    const std::vector<std::vector<bool>>&   getAdjacency() const { return adjacency_; }
    /** Optional shared-face vertex lists keyed by (i,j) — may be empty unless populated. */
    const SharedFaceMap&                    getSharedFaces() const { return sharedFaceVertices_; }

    /** Indices of the cells that contain start/goal, or -1 if outside. */
    int getStartCell() const { return startCell_; }
    int getGoalCell() const { return goalCell_; }

    const std::array<double, 6>& getBounds() const { return bounds_; }

    const std::vector<ObstacleMesh>& getObstacles() const { return obstacles_; }

    int getDimension() const { return dim_; }

private:

    // --- Helpers -----------------------------------------------------------------------------

    void clearComputedData_();

    struct CGALImpl;
    
    // --- Members -----------------------------------------------------------------------------

    ompl::base::SpaceInformationPtr si_;  // OMPL SI (state space / allocator, etc.)
    std::unique_ptr<CGALImpl> impl_;      // CGAL internals
    int dim_{3};                          // 2 or 3 (affects z handling)

    // Input Data
    std::array<double, 6> bounds_{{0, 1, 0, 1, 0, 1}};
    std::vector<ObstacleMesh> obstacles_;
    std::array<double, 3> start_{{0, 0, 0}};
    std::array<double, 3> goal_{{0, 0, 0}};

    // Derived state (after performConvexDecomposition)
    std::vector<ConvexCell> cells_;
    std::vector<std::vector<bool>> adjacency_;
    SharedFaceMap sharedFaceVertices_;
    int startCell_{-1};
    int goalCell_{-1};
};

}}} // namespace ompl::geometric::informedhz

#endif // OMPL_GEOMETRIC_PLANNERS_RRT_INFORMEDHZ_HYBRID_ZONOTOPE_H