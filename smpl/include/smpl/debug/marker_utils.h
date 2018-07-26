#ifndef SMPL_MARKER_UTILS_H
#define SMPL_MARKER_UTILS_H

#include <smpl/debug/marker.h>

namespace sbpl {
namespace visual {

auto MakeEmptyMarker() -> Marker;

auto MakeSphereMarker(
    double x, double y, double z,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeLineMarker(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0)
    -> Marker;

auto MakeFrameMarkers(
    const Eigen::Affine3d& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0)
    -> std::vector<Marker>;

auto MakePoseMarkers(
    const Eigen::Affine3d& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0,
    bool text = false)
    -> std::vector<Marker>;

auto MakeCubesMarker(
    const std::vector<Eigen::Vector3d>& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubesMarker(
    const std::vector<Eigen::Vector3d>& centers,
    double size,
    const std::vector<Color>& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubesMarker(
    std::vector<Eigen::Vector3d>&& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubesMarker(
    std::vector<Eigen::Vector3d>&& centers,
    double size,
    std::vector<Color>&& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

} // namespace visual
} // namespace sbpl

#endif
