#ifndef SMPL_URDF_ROBOT_MODEL_ROBOT_MODEL_H
#define SMPL_URDF_ROBOT_MODEL_ROBOT_MODEL_H

// standard includes
#include <cstddef>

namespace urdf {
class ModelInterface;
} // namespace urdf

// project includes
#include <smpl_urdf_robot_model/array_range.h>
#include <smpl_urdf_robot_model/types.h>

namespace sbpl {
namespace motion {
namespace urdf {

struct RobotModel;
struct JointSpec;
struct Link;
struct Joint;
struct JointVariable;
struct LinkCollision;
struct LinkVisual;
struct VariableLimits;

bool InitRobotModel(
    RobotModel* robot_model,
    const ::urdf::ModelInterface* urdf,
    const JointSpec* world_joint = NULL);

auto GetName(const RobotModel* model) -> const std::string*;

auto GetRootJoint(const RobotModel* model) -> const Joint*;
auto GetRootLink(const RobotModel* model) -> const Link*;

///////////////
// Iteration //
///////////////

auto GetLinkCount(const RobotModel* model) -> size_t;
auto GetJointCount(const RobotModel* model) -> size_t;
auto GetVariableCount(const RobotModel* model) -> size_t;
auto GetCollisionBodyCount(const RobotModel* model) -> size_t;
auto GetVisualBodyCount(const RobotModel* model) -> size_t;

auto GetLink(const RobotModel* model, int index) -> const Link*;
auto GetJoint(const RobotModel* model, int index) -> const Joint*;
auto GetVariable(const RobotModel* model, int index) -> const JointVariable*;
auto GetCollisionBody(const RobotModel* model, int index) -> const LinkCollision*;
auto GetVisualBody(const RobotModel* model, int index) -> const LinkVisual*;

auto Links(const RobotModel* model) -> range<const Link*>;
auto Joints(const RobotModel* model) -> range<const Joint*>;
auto Variables(const RobotModel* model) -> range<const JointVariable*>;
auto CollisionBodies(const RobotModel* model) -> range<const LinkCollision*>;
auto VisualBodies(const RobotModel* model) -> range<const LinkVisual*>;

auto GetLinkIndex(const RobotModel* model, const Link* link) -> size_t;
auto GetJointIndex(const RobotModel* model, const Joint* joint) -> size_t;
auto GetVariableIndex(const RobotModel* model, const JointVariable* variable) -> size_t;
auto GetCollisionBodyIndex(const RobotModel* model, const LinkCollision* collision) -> size_t;
auto GetVisualBodyIndex(const RobotModel* model, const LinkVisual* visual) -> size_t;

///////////
// Names //
///////////

auto GetName(const Link* link) -> const std::string*;
auto GetName(const Joint* joint) -> const std::string*;
auto GetName(const JointVariable* variable) -> const std::string*;

auto GetLinkName(const RobotModel* model, int index) -> const std::string*;
auto GetJointName(const RobotModel* model, int index) -> const std::string*;
auto GetVariableName(const RobotModel* model, int index) -> const std::string*;

auto GetLink(const RobotModel* model, const char* name) -> const Link*;
auto GetJoint(const RobotModel* model, const char* name) -> const Joint*;
auto GetVariable(const RobotModel* model, const char* name) -> const JointVariable*;
auto GetLink(const RobotModel* model, const std::string* name) -> const Link*;
auto GetJoint(const RobotModel* model, const std::string* name) -> const Joint*;
auto GetVariable(const RobotModel* model, const std::string* name) -> const JointVariable*;

/////////////////////
// Link Properties //
/////////////////////

auto VisualBodies(const Link* link) -> range<const LinkVisual*>;
auto CollisionBodies(const Link* link) -> range<const LinkCollision*>;

//////////////////////
// Joint Properties //
//////////////////////

auto GetVariableCount(const Joint* joint) -> size_t;
auto GetFirstVariable(const Joint* joint) -> const JointVariable*;
auto Variables(const Joint* joint) -> range<const JointVariable*>;

auto GetCommonRoot(const RobotModel* model, const Joint* a, const Joint* b) -> const Joint*;
bool IsAncestor(const RobotModel* model, const Joint* a, const Joint* b);

/////////////////////////
// Variable Properties //
/////////////////////////

auto GetJointOfVariable(const JointVariable* variable) -> const Joint*;
auto GetVariableLimits(const JointVariable* variable) -> const VariableLimits*;
auto GetDefaultPosition(const RobotModel* model, const JointVariable* variable) -> double;

/////////////////
// Definitions //
/////////////////

enum struct ShapeType
{
    Sphere,
    Box,
    Cylinder,
    Mesh
};

struct Shape
{
    ShapeType type;
};

struct Sphere : Shape
{
    double radius;
    Sphere() { type = ShapeType::Sphere; }
};

struct Box : Shape
{
    Vector3 size;
    Box() { type = ShapeType::Box; }
};

struct Cylinder : Shape
{
    double height;
    double radius;
    Cylinder() { type = ShapeType::Cylinder; }
};

struct Mesh : Shape
{
    std::string filename;
    Vector3     scale;
    Mesh() { type = ShapeType::Mesh; }
};

struct LinkVisual
{
    Affine3 origin;
    Shape*  shape = NULL;
    Link*   link = NULL;
};

struct LinkCollision
{
    Affine3 origin;
    Shape*  shape = NULL;
    Link*   link = NULL;
};

struct Link
{
    std::string name;
    Joint*      parent = NULL;
    Joint*      children = NULL;

    range<LinkVisual*>    visual;
    range<LinkCollision*> collision;
};

struct VariableLimits
{
    bool has_position_limits = false;
    double min_position = 0.0;
    double max_position = 0.0;
    double max_velocity = 0.0;
    double max_effort = 0.0;
};

enum struct JointType
{
    Fixed,      // 0 DOF
    Revolute,   // 1 DOF, rotates around axis
    Prismatic,  // 1 DOF, translates along axis
    Planar,     // 3 DOF planar motion
    Floating,   // 6 DOF rigid body motion
};

struct Joint
{
    std::string name;

    JointType type;
    Affine3 origin;
    Vector3 axis;   // the axis for revolute and prismatic joints

    JointVariable* vfirst = NULL;
    JointVariable* vlast = NULL;

    Link* child = NULL;
    Link* parent = NULL;
    Joint* sibling = NULL;
};

struct JointVariable
{
    std::string name;
    VariableLimits limits;
    Joint* joint = NULL;
};

// A robot is described as a set of links, rigid bodies in space, constrained
// to one another by joints, which specify the allowable relative motion between
// them. The position of each joint is parameterized by one or more variables
struct RobotModel
{
    std::string name;

    std::vector<Link> links;

    std::vector<Joint> joints;

    std::vector<JointVariable> variables;

    std::vector<Sphere>     spheres;
    std::vector<Box>        boxes;
    std::vector<Cylinder>   cylinders;
    std::vector<Mesh>       meshes;

    std::vector<LinkCollision>  collisions;
    std::vector<LinkVisual>     visuals;

    Link* root_link = NULL;
    Joint* root_joint = NULL;

    std::vector<const Joint*> ancestor_map;

    // self-references => non-copyable
    RobotModel() = default;
    RobotModel(const RobotModel&) = delete;
    RobotModel(RobotModel&&) = default;
    RobotModel& operator=(const RobotModel&) = delete;
    RobotModel& operator=(RobotModel&&) = default;
};

struct JointSpec
{
    Affine3 origin;
    Vector3 axis;
    std::string name;
    JointType type;
};

} // namespace urdf
} // namespace motion
} // namespace smpl

#endif

