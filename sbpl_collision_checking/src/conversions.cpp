#include <sbpl_collision_checking/conversions.h>

#include <assert.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

namespace smpl {
namespace collision {

// Construct a CollisionShape from a SolidPrimitive. The returned shape is
// expected to be freed by the caller.
static auto ConstructShapeFromMsg(const shape_msgs::SolidPrimitive& prim)
    -> CollisionShape*
{
    switch (prim.type) {
    case shape_msgs::SolidPrimitive::BOX:
    {
        auto size_x = prim.dimensions[shape_msgs::SolidPrimitive::BOX_X];
        auto size_y = prim.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
        auto size_z = prim.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
        return new BoxShape(size_x, size_y, size_z);
    }
    case shape_msgs::SolidPrimitive::SPHERE:
    {
        auto radius = prim.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
        return new SphereShape(radius);
    }
    case shape_msgs::SolidPrimitive::CYLINDER:
    {
        auto radius = prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
        auto height = prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
        return new CylinderShape(radius, height);
    }
    case shape_msgs::SolidPrimitive::CONE:
    {
        auto radius = prim.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS];
        auto height = prim.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT];
        return new ConeShape(radius, height);
    }
    default:
        return NULL;
    }
}

// Construct a CollisionShape from a Mesh. Deep copies of the vertex and index
// data are made. Both the returned mesh shape and the associated vertex and
// index are expected to be freed by the caller.
static auto ConstructShapeFromMsg(const shape_msgs::Mesh& mesh)
    -> CollisionShape*
{
    auto vertices = new double[3 * mesh.vertices.size()];
    auto triangles = new std::uint32_t[3 * mesh.triangles.size()];

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        vertices[3 * i + 0] = mesh.vertices[i].x;
        vertices[3 * i + 1] = mesh.vertices[i].y;
        vertices[3 * i + 2] = mesh.vertices[i].z;
    }

    for (size_t i = 0; i < mesh.triangles.size(); ++i) {
        triangles[3 * i + 0] = mesh.triangles[i].vertex_indices[0];
        triangles[3 * i + 1] = mesh.triangles[i].vertex_indices[1];
        triangles[3 * i + 2] = mesh.triangles[i].vertex_indices[2];
    }

    auto* m = new MeshShape;
    m->vertices = vertices;
    m->triangles = triangles;
    m->vertex_count = mesh.vertices.size();
    m->triangle_count = mesh.triangles.size();
    return m;
}

// Construct a CollisionShape from a Plane. The returned shape is expected to be
// freed by the caller.
static auto ConstructShapeFromMsg(const shape_msgs::Plane& plane)
    -> CollisionShape*
{
    auto* p = new PlaneShape;
    p->a = plane.coef[0];
    p->b = plane.coef[1];
    p->c = plane.coef[2];
    p->d = plane.coef[3];
    return p;
}

static auto MakeCollisionShapeShallow(const shapes::Shape* shape)
    -> CollisionShape*
{
    switch (shape->type) {
    case shapes::UNKNOWN_SHAPE:
        return NULL;
    case shapes::SPHERE:
    {
        auto* s = static_cast<const shapes::Sphere*>(shape);
        return new SphereShape(s->radius);
    }
    case shapes::CYLINDER:
    {
        auto* s = static_cast<const shapes::Cylinder*>(shape);
        return new CylinderShape(s->radius, s->length);
    }
    case shapes::CONE:
    {
        auto* s = static_cast<const shapes::Cone*>(shape);
        return new ConeShape(s->radius, s->length);
    }
    case shapes::BOX:
    {
        auto* s = static_cast<const shapes::Box*>(shape);
        return new BoxShape(s->size[0], s->size[1], s->size[2]);
    }
    case shapes::PLANE:
    {
        auto* s = static_cast<const shapes::Plane*>(shape);
        return new PlaneShape(s->a, s->b, s->c, s->d);
    }
    case shapes::MESH:
    {
        auto* s = static_cast<const shapes::Mesh*>(shape);
        auto* ms = new MeshShape;
        ms->vertices = s->vertices;
        ms->vertex_count = s->vertex_count;
        ms->triangles = s->triangles;
        ms->triangle_count = s->triangle_count;
        return ms;
    }
    case shapes::OCTREE:
    {
        auto* s = static_cast<const shapes::OcTree*>(shape);
        return new OcTreeShape(s->octree.get());
    }
    default:
        return NULL;
    }
}

static auto MakeCollisionShapeDeep(const shapes::Shape* shape)
    -> CollisionShape*
{
    if (shape->type == shapes::MESH) {
        auto* s = static_cast<const shapes::Mesh*>(shape);

        auto vertices = new double[3 * s->vertex_count];
        auto triangles = new std::uint32_t[3 * s->triangle_count];

        std::copy(s->vertices, s->vertices + 3 * s->vertex_count, vertices);
        std::copy(s->triangles, s->triangles + 3 * s->triangle_count, triangles);

        auto* ms = new MeshShape;
        ms->vertices = vertices;
        ms->vertex_count = s->vertex_count;
        ms->triangles = triangles;
        ms->triangle_count = s->triangle_count;
        return ms;
    } else if (shape->type == shapes::OCTREE) {
        auto* s = static_cast<const shapes::OcTree*>(shape);
        auto* octree = new octomap::OcTree(*s->octree);
        return new OcTreeShape(octree);
    } else {
        return MakeCollisionShapeShallow(shape);
    }
}

// Construct a shapes::Shape from a CollisionShape. The CollisionShape must
// be of a known type. The data associated with the CollisionShape is deep
// copied to be managed by the returned shapes::Shape.
static auto MakeShape(const CollisionShape* shape)
    -> shapes::ShapeConstPtr
{
    switch (shape->type) {
    case ShapeType::Sphere:
    {
        auto* s = static_cast<const SphereShape*>(shape);
        return shapes::ShapeConstPtr(new shapes::Sphere(s->radius));
    }
    case ShapeType::Cylinder:
    {
        auto* s = static_cast<const CylinderShape*>(shape);
        return shapes::ShapeConstPtr(new shapes::Cylinder(s->radius, s->height));
    }
    case ShapeType::Cone:
    {
        auto* s = static_cast<const ConeShape*>(shape);
        return shapes::ShapeConstPtr(new shapes::Cone(s->radius, s->height));
    }
    case ShapeType::Box:
    {
        auto* s = static_cast<const BoxShape*>(shape);
        return shapes::ShapeConstPtr(new shapes::Box(s->size[0], s->size[1], s->size[2]));
    }
    case ShapeType::Plane:
    {
        auto* s = static_cast<const PlaneShape*>(shape);
        return shapes::ShapeConstPtr(new shapes::Plane(s->a, s->b, s->c, s->d));
    }
    case ShapeType::Mesh:
    {
        auto* s = static_cast<const MeshShape*>(shape);

        auto vertices = new double[3 * s->vertex_count];
        auto triangles = new std::uint32_t[3 * s->triangle_count];

        std::copy(s->vertices, s->vertices + 3 * s->vertex_count, vertices);
        std::copy(s->triangles, s->triangles + 3 * s->triangle_count, triangles);

        auto ms = new shapes::Mesh;
        ms->vertices = vertices;
        ms->vertex_count = s->vertex_count;
        ms->triangles = triangles;
        ms->triangle_count = s->triangle_count;
        return shapes::ShapeConstPtr(ms);
    }
    case ShapeType::OcTree:
    {
        auto* s = static_cast<const OcTreeShape*>(shape);

        auto octree = decltype(shapes::OcTree().octree)(
                new octomap::OcTree(*s->octree));

        return shapes::ShapeConstPtr(new shapes::OcTree(octree));
    }
    default:
        return NULL;
    }
}

// moveit_msgs::CollisionObject -> collision_detection::World::Object
void ConvertCollisionObjectMsgToWorldObject(
    const moveit_msgs::CollisionObject& in,
    collision_detection::World::Object& out)
{
    assert(in.primitives.size() == in.primitive_poses.size());
    assert(in.meshes.size() == in.mesh_poses.size());
    assert(in.planes.size() == in.plane_poses.size());

    collision_detection::World::Object o(in.id);

    auto shape_count =
            in.planes.size() + in.primitives.size() + in.meshes.size();
    out.shapes_.reserve(shape_count);
    out.shape_poses_.reserve(shape_count);

    for (size_t pidx = 0; pidx < in.primitives.size(); ++pidx) {
        auto& prim = in.primitives[pidx];
        auto& pose = in.primitive_poses[pidx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(prim));
        if (!sp) {
            ROS_WARN("Failed to construct shape from primitive message");
            continue;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o.shapes_.push_back(sp);
        o.shape_poses_.push_back(transform);
    }

    for (size_t midx = 0; midx < in.meshes.size(); ++midx) {
        auto& mesh = in.meshes[midx];
        auto& pose = in.mesh_poses[midx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(mesh));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from mesh message");
            continue;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o.shapes_.push_back(sp);
        o.shape_poses_.push_back(transform);
    }

    for (size_t pidx = 0; pidx < in.planes.size(); ++pidx) {
        auto& plane = in.planes[pidx];
        auto& pose = in.plane_poses[pidx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(plane));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from plane message");
            continue;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o.shapes_.push_back(sp);
        o.shape_poses_.push_back(transform);
    }

    out = std::move(o);
}

// moveit_msgs::CollisionObject -> scdl::CollisionObject
void ConvertCollisionObjectMsgToCollisionObject(
    const moveit_msgs::CollisionObject& in,
    CollisionObject& out)
{
    assert(in.primitives.size() == in.primitive_poses.size());
    assert(in.meshes.size() == in.mesh_poses.size());
    assert(in.planes.size() == in.plane_poses.size());

    CollisionObject o;
    o.id = in.id;

    auto shape_count =
            in.planes.size() + in.primitives.size() + in.meshes.size();
    out.shapes.reserve(shape_count);
    out.shape_poses.reserve(shape_count);

    for (size_t pidx = 0; pidx < in.primitives.size(); ++pidx) {
        auto& prim = in.primitives[pidx];
        auto& pose = in.primitive_poses[pidx];

        auto* sp = ConstructShapeFromMsg(prim);
        if (!sp) {
            ROS_WARN("Failed to construct shape from primitive message");
            continue;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o.shapes.push_back(sp);
        o.shape_poses.push_back(transform);
    }

    for (size_t midx = 0; midx < in.meshes.size(); ++midx) {
        auto& mesh = in.meshes[midx];
        auto& pose = in.mesh_poses[midx];

        auto* sp = ConstructShapeFromMsg(mesh);
        if (!sp) {
            ROS_ERROR("Failed to construct shape from mesh message");
            continue;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o.shapes.push_back(sp);
        o.shape_poses.push_back(transform);
    }

    for (size_t pidx = 0; pidx < in.planes.size(); ++pidx) {
        auto& plane = in.planes[pidx];
        auto& pose = in.plane_poses[pidx];

        auto* sp = ConstructShapeFromMsg(plane);
        if (!sp) {
            ROS_ERROR("Failed to construct shape from plane message");
            continue;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o.shapes.push_back(sp);
        o.shape_poses.push_back(transform);
    }

    out = std::move(o);
}

// collision_detection::World -> moveit_msgs::CollisionObject
void ConvertWorldObjectToCollisionObjectMsg(
    const collision_detection::World::Object& in,
    moveit_msgs::CollisionObject& out)
{
    assert(in.shapes_.size() == in.shape_poses_.size());

    moveit_msgs::CollisionObject o;

    for (size_t i = 0; i < in.shapes_.size(); ++i) {
        auto& shape = in.shapes_[i];
        auto& shape_pose = in.shape_poses_[i];

        shapes::ShapeMsg msg;
        if (!shapes::constructMsgFromShape(shape.get(), msg)) {
            ROS_WARN("Failed to construct shape message from shape");
            continue;
        }

        struct visitor : public boost::static_visitor<void>
        {
            moveit_msgs::CollisionObject& o;
            geometry_msgs::Pose pose;

            visitor(
                moveit_msgs::CollisionObject& o,
                geometry_msgs::Pose pose)
            :
                o(o),
                pose(pose)
            { }

            void operator()(const shape_msgs::SolidPrimitive& prim)
            {
                o.primitives.push_back(prim);
                o.primitive_poses.push_back(pose);
            }
            void operator()(const shape_msgs::Mesh& mesh)
            {
                o.meshes.push_back(mesh);
                o.mesh_poses.push_back(pose);
            }
            void operator()(const shape_msgs::Plane& plane)
            {
                o.planes.push_back(plane);
                o.plane_poses.push_back(pose);
            }
        };

        geometry_msgs::Pose p;
        tf::poseEigenToMsg(shape_pose, p);

        auto vis = visitor(o, p);
        boost::apply_visitor(vis, msg);
    }

    out = std::move(o);
}

// collision_detection::World::Object -> scdl::CollisionObject
void ConvertWorldObjectToCollisionObject(
    const collision_detection::World::Object& in,
    CollisionObject& out)
{
    assert(in.shapes_.size() == in.shape_poses_.size());

    CollisionObject o;
    o.id = in.id_;

    o.shapes.reserve(in.shapes_.size());
    o.shape_poses.reserve(in.shapes_.size());
    for (size_t i = 0; i < in.shapes_.size(); ++i) {
        auto& shape = in.shapes_[i];
        auto& pose = in.shape_poses_[i];

        o.shapes.push_back(MakeCollisionShapeDeep(shape.get()));
        o.shape_poses.push_back(pose);
    }

    out = std::move(o);
}

void ConvertWorldObjectToCollisionObjectShallow(
    const collision_detection::World::Object& in,
    CollisionObject& out)
{
    assert(in.shapes_.size() == in.shape_poses_.size());

    CollisionObject o;
    o.id = in.id_;

    o.shapes.reserve(in.shapes_.size());
    o.shape_poses.reserve(in.shapes_.size());
    for (size_t i = 0; i < in.shapes_.size(); ++i) {
        auto& shape = in.shapes_[i];
        auto& pose = in.shape_poses_[i];

        o.shapes.push_back(MakeCollisionShapeShallow(shape.get()));
        o.shape_poses.push_back(pose);
    }

    out = std::move(o);
}

// scdl::CollisionObject -> moveit_msgs::CollisionObject
void ConvertCollisionObjectToCollisionObjectMsg(
    const CollisionObject& in,
    moveit_msgs::CollisionObject& out)
{
    collision_detection::World::Object tmp(in.id);
    ConvertCollisionObjectToWorldObject(in, tmp);
    ConvertWorldObjectToCollisionObjectMsg(tmp, out);
}

// scdl::CollisionObject -> collision_detection::World::Object
void ConvertCollisionObjectToWorldObject(
    const CollisionObject& in,
    collision_detection::World::Object& out)
{
    assert(in.shapes.size() == in.shape_poses.size());

    collision_detection::World::Object o(in.id);

    for (size_t i = 0; i < in.shapes.size(); ++i) {
        auto& shape = in.shapes[i];
        auto& pose = in.shape_poses[i];

        assert(0); // TODO:
    }

    out = std::move(o);
}

void FreeManagedCollisionObject(CollisionObject& co)
{
    for (auto& shape : co.shapes) {
        switch (shape->type) {
        case smpl::collision::ShapeType::Sphere:
            delete static_cast<smpl::collision::SphereShape*>(shape);
            break;
        case smpl::collision::ShapeType::Cylinder:
            delete static_cast<smpl::collision::CylinderShape*>(shape);
            break;
        case smpl::collision::ShapeType::Cone:
            delete static_cast<smpl::collision::ConeShape*>(shape);
            break;
        case smpl::collision::ShapeType::Box:
            delete static_cast<smpl::collision::BoxShape*>(shape);
            break;
        case smpl::collision::ShapeType::Plane:
            delete static_cast<smpl::collision::PlaneShape*>(shape);
            break;
        case smpl::collision::ShapeType::Mesh:
        {
            auto* m = static_cast<smpl::collision::MeshShape*>(shape);
            if (m->vertices != NULL) {
                delete[] m->vertices;
            }
            if (m->triangles != NULL) {
                delete[] m->triangles;
            }
            delete m;
            break;
        }
        case smpl::collision::ShapeType::OcTree:
        {
            auto* o = static_cast<smpl::collision::OcTreeShape*>(shape);
            delete o->octree;
            delete o;
            break;
        }
        }
    }

    co.shapes.clear();
    co.shape_poses.clear();
}

void FreeManagedCollisionObjectShallow(CollisionObject& co)
{
    for (auto& shape : co.shapes) {
        switch (shape->type) {
        case smpl::collision::ShapeType::Sphere:
            delete static_cast<smpl::collision::SphereShape*>(shape);
            break;
        case smpl::collision::ShapeType::Cylinder:
            delete static_cast<smpl::collision::CylinderShape*>(shape);
            break;
        case smpl::collision::ShapeType::Cone:
            delete static_cast<smpl::collision::ConeShape*>(shape);
            break;
        case smpl::collision::ShapeType::Box:
            delete static_cast<smpl::collision::BoxShape*>(shape);
            break;
        case smpl::collision::ShapeType::Plane:
            delete static_cast<smpl::collision::PlaneShape*>(shape);
            break;
        case smpl::collision::ShapeType::Mesh:
            delete static_cast<smpl::collision::MeshShape*>(shape);
            break;
        case smpl::collision::ShapeType::OcTree:
            delete static_cast<smpl::collision::OcTreeShape*>(shape);
            break;
        }
    }

    co.shapes.clear();
    co.shape_poses.clear();
}

} // namespace collision
} // namespace smpl
