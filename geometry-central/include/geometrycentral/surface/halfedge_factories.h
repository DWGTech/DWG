#pragma once

#include "geometrycentral/surface/surface_mesh_factories.h"

#include <memory>
#include <tuple>

// NOTE these are DEPRECATED! Prefer the versions in surface_mesh_factories.h

// These methods construct the connectivity and geometry of a mesh simultaneously.

namespace geometrycentral {
namespace surface {

// Assumes manifoldness, errors our if not
std::tuple<std::unique_ptr<ManifoldSurfaceMesh>, std::unique_ptr<VertexPositionGeometry>>
makeHalfedgeAndGeometry(const std::vector<std::vector<size_t>>& polygons, const std::vector<Vector3> vertexPositions);


// Like above, but with known twin connectivity
std::tuple<std::unique_ptr<ManifoldSurfaceMesh>, std::unique_ptr<VertexPositionGeometry>>
makeHalfedgeAndGeometry(const std::vector<std::vector<size_t>>& polygons,
                        const std::vector<std::vector<std::tuple<size_t, size_t>>>& twins,
                        const std::vector<Vector3> vertexPositions);


// Same a above, but constructs a potentially-nonmanifold surface mesh
std::tuple<std::unique_ptr<SurfaceMesh>, std::unique_ptr<VertexPositionGeometry>>
makeGeneralHalfedgeAndGeometry(const std::vector<std::vector<size_t>>& polygons,
                               const std::vector<Vector3> vertexPositions);


// Like above, but with known twin connectivity
std::tuple<std::unique_ptr<SurfaceMesh>, std::unique_ptr<VertexPositionGeometry>>
makeGeneralHalfedgeAndGeometry(const std::vector<std::vector<size_t>>& polygons,
                               const std::vector<std::vector<std::tuple<size_t, size_t>>>& twins,
                               const std::vector<Vector3> vertexPositions);

} // namespace surface
} // namespace geometrycentral
