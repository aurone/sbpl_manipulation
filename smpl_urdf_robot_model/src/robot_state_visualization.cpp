#include <smpl_urdf_robot_model/robot_state_visualization.h>

// system includes
#include <smpl/console/console.h>

// project includes
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>

namespace sbpl {
namespace motion {
namespace urdf {

static
auto MakeShapeVisualization(const Shape* shape) -> sbpl::visual::Shape
{
    switch (shape->type) {
    case ShapeType::Sphere:
    {
        auto* tmp = static_cast<const Sphere*>(shape);
        return sbpl::visual::Sphere{ tmp->radius };
    }
    case ShapeType::Box:
    {
        auto* tmp = static_cast<const Box*>(shape);
        return sbpl::visual::Cube{ tmp->size.x(), tmp->size.y(), tmp->size.z() };

    }
    case ShapeType::Cylinder:
    {
        auto* tmp = static_cast<const Cylinder*>(shape);
        return sbpl::visual::Cylinder{ tmp->radius, tmp->height };
    }
    case ShapeType::Mesh:
    {
        auto* tmp = static_cast<const Mesh*>(shape);
        return sbpl::visual::MeshResource{ tmp->filename, tmp->scale };
    }
    }
}

auto MakeRobotVisualization(
    const RobotState* state,
    sbpl::visual::Color color,
    const std::string& frame,
    const std::string& ns,
    int32_t* id)
    -> std::vector<sbpl::visual::Marker>
{
    std::vector<sbpl::visual::Marker> markers;

    auto first_id = id == NULL ? 0 : *id;
    for (auto& link : Links(state->model)) {
        auto* pose = GetLinkTransform(state, &link);
        for (auto& visual : link.visual) {
            sbpl::visual::Marker m;

            m.pose = *GetVisualBodyTransform(state, &visual);
            m.shape = MakeShapeVisualization(visual.shape);
            m.color = color;
            m.frame_id = frame;
            m.ns = ns;
            m.lifetime = 0;
            m.id = first_id++;
            markers.push_back(m);
        }
    }

    if (id != NULL) *id = first_id;
    return markers;
}

auto MakeCollisionVisualization(
    const RobotState* state,
    sbpl::visual::Color color,
    const std::string& frame,
    const std::string& ns,
    int32_t* id)
    -> std::vector<sbpl::visual::Marker>
{
    std::vector<sbpl::visual::Marker> markers;

    auto first_id = id == NULL ? 0 : *id;
    for (auto& link : Links(state->model)) {
        for (auto& collision : link.collision) {
            sbpl::visual::Marker m;
            m.pose = *GetCollisionBodyTransform(state, &collision);
            m.shape = MakeShapeVisualization(collision.shape);
            m.color = color;
            m.frame_id = frame;
            m.ns = ns;
            m.lifetime = 0;
            m.id = first_id++;
            markers.push_back(m);
        }
    }

    if (id != NULL) *id = first_id;

    return markers;
}

} // namespace urdf
} // namespace motion
} // namesace sbpl

