#include <smpl_urdf_robot_model/robot_model.h>

#include <urdf_model/model.h>

namespace sbpl {
namespace motion {
namespace urdf {

static
auto PoseURDFToEigen(const ::urdf::Pose& p) -> Affine3
{
    return Translation3(p.position.x, p.position.y, p.position.z) *
            Quaternion(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
}

static
auto VectorURDFToEigen(const ::urdf::Vector3& v) -> Vector3
{
    return Vector3(v.x, v.y, v.z);
}

static
void AddPlanarJointVariables(
    const std::string* name,
    std::vector<JointVariable>* variables)
{
    JointVariable v;
    v.limits.has_position_limits = false;
    v.limits.min_position = -std::numeric_limits<double>::infinity();
    v.limits.max_position = std::numeric_limits<double>::infinity();
    v.limits.max_velocity = 0.0;
    v.limits.max_effort = 0.0;

    v.name = (*name) + "/x";
    variables->push_back(v);

    v.name = (*name) + "/y";
    variables->push_back(v);

    v.name = (*name) + "/theta";
    variables->push_back(v);
}

static
void AddFloatingJointVariables(
    const std::string* name,
    std::vector<JointVariable>* variables)
{
    JointVariable v;

    v.limits.has_position_limits = false;
    v.limits.min_position = -std::numeric_limits<double>::infinity();
    v.limits.max_position = std::numeric_limits<double>::infinity();
    v.limits.max_velocity = 0.0;
    v.limits.max_effort = 0.0;

    v.name = (*name) + "/trans_x";
    variables->push_back(v);

    v.name = (*name) + "/trans_y";
    variables->push_back(v);

    v.name = (*name) + "/trans_z";
    variables->push_back(v);

    v.name = (*name) + "/rot_x";
    variables->push_back(v);

    v.name = (*name) + "/rot_y";
    variables->push_back(v);

    v.name = (*name) + "/rot_z";
    variables->push_back(v);

    v.name = (*name) + "/rot_w";
    variables->push_back(v);
}

bool InitRobotModel(
    RobotModel* out,
    const ::urdf::ModelInterface* urdf,
    const JointSpec* world_joint)
{
    RobotModel robot_model;

    robot_model.name = urdf->getName();
    robot_model.joints.reserve(urdf->joints_.size() + 1);
    robot_model.links.reserve(urdf->links_.size());
    robot_model.variables.reserve(urdf->joints_.size() + 7); // common case

    ////////////////////////////////
    // Initialize link properties //
    ////////////////////////////////

    // ...init basic link properties
    for (auto& e : urdf->links_) {
        Link link;
        link.name = e.first;
        robot_model.links.push_back(std::move(link));

        // ...gather shapes in the meantime
        auto add_shape = [&](const ::urdf::Geometry* geom)
        {
            switch (geom->type) {
            case ::urdf::Geometry::SPHERE:
            {
                auto* s = static_cast<const ::urdf::Sphere*>(geom);
                Sphere sphere;
                sphere.radius = s->radius;
                robot_model.spheres.push_back(sphere);
                break;
            }
            case ::urdf::Geometry::BOX:
            {
                auto* b = static_cast<const ::urdf::Box*>(geom);
                Box box;
                box.size = VectorURDFToEigen(b->dim);
                robot_model.boxes.push_back(box);
                break;
            }
            case ::urdf::Geometry::CYLINDER:
            {
                auto* c = static_cast<const ::urdf::Cylinder*>(geom);
                Cylinder cylinder;
                cylinder.height = c->length;
                cylinder.radius = c->radius;
                robot_model.cylinders.push_back(cylinder);
                break;
            }
            case ::urdf::Geometry::MESH:
            {
                auto* m = static_cast<const ::urdf::Mesh*>(geom);
                Mesh mesh;
                mesh.filename = m->filename;
                mesh.scale = Vector3(m->scale.x, m->scale.y, m->scale.z);
                robot_model.meshes.push_back(mesh);
                break;
            }
            }
        };

        if (!e.second->collision_array.empty()) {
            for (auto& collision : e.second->collision_array) {
                add_shape(collision->geometry.get());
                LinkCollision c;
                c.origin = PoseURDFToEigen(collision->origin);
                robot_model.collisions.push_back(c);
            }
        } else if (e.second->collision) {
            add_shape(e.second->collision->geometry.get());
            LinkCollision c;
            c.origin = PoseURDFToEigen(e.second->collision->origin);
            robot_model.collisions.push_back(c);
        }

        if (!e.second->visual_array.empty()) {
            for (auto& visual : e.second->visual_array) {
                add_shape(visual->geometry.get());
                LinkVisual c;
                c.origin = PoseURDFToEigen(e.second->visual->origin);
                robot_model.visuals.push_back(c);
            }
        } else if (e.second->visual) {
            add_shape(e.second->visual->geometry.get());
            LinkVisual c;
            c.origin = PoseURDFToEigen(e.second->visual->origin);
            robot_model.visuals.push_back(c);
        }
    }

    auto get_link = [](RobotModel* model, const std::string& name) -> Link* {
        for (auto& link : model->links) {
            if (link.name == name) {
                return &link;
            }
        }
        return NULL;
    };

    // ...another pass to hook up link visual/collision geometry
    auto sphere_index = 0;
    auto box_index = 0;
    auto cylinder_index = 0;
    auto mesh_index = 0;
    auto* prev_collision = robot_model.collisions.data();
    auto* prev_visual = robot_model.visuals.data();
    for (auto& e : urdf->links_) {
        auto* link = get_link(&robot_model, e.first);

        auto* c = prev_collision;
        auto* v = prev_visual;
        auto next_shape = [&](const ::urdf::Geometry* geom) -> Shape*
        {
            switch (geom->type) {
            case ::urdf::Geometry::SPHERE:
                return &robot_model.spheres[sphere_index++];
            case ::urdf::Geometry::BOX:
                return &robot_model.boxes[box_index++];
            case ::urdf::Geometry::CYLINDER:
                return &robot_model.cylinders[cylinder_index++];
            case ::urdf::Geometry::MESH:
                return &robot_model.meshes[mesh_index++];
            }
        };

        // ...assign shapes and parenting links to all collision/visual instances

        if (!e.second->collision_array.empty()) {
            for (auto& collision : e.second->collision_array) {
                c->shape = next_shape(collision->geometry.get());
                ++c;
                c->link = link;
            }
        } else if (e.second->collision) {
            c->shape = next_shape(e.second->collision->geometry.get());
            ++c;
            c->link = link;
        }

        if (!e.second->visual_array.empty()) {
            for (auto& collision : e.second->visual_array) {
                v->shape = next_shape(collision->geometry.get());
                ++v;
                v->link = link;
            }
        } else if (e.second->collision) {
            v->shape = next_shape(e.second->collision->geometry.get());
            ++v;
            v->link = link;
        }

        // ...assign range of collision/visual instances to each link

        link->collision = make_range(prev_collision, c);
        link->visual = make_range(prev_visual, v);

        prev_collision = c;
        prev_visual = v;
    }

    /////////////////////////////////
    // Initialize joint properties //
    /////////////////////////////////

    // ...for the world joint
    Joint wj;
    if (world_joint != NULL) {
        wj.name = world_joint->name;
        wj.origin = world_joint->origin;
        wj.axis = world_joint->axis;
        wj.type = world_joint->type;
    } else {
        wj.name = "default_world_joint";
        wj.origin = Affine3::Identity();
        wj.axis = Vector3::Zero();
        wj.type = JointType::Floating;
    }
    robot_model.joints.push_back(wj);

    // ...for the joints in the ::urdf
    for (auto& e : urdf->joints_) {
        Joint joint;

        joint.name = e.first;
        joint.origin = PoseURDFToEigen(e.second->parent_to_joint_origin_transform);
        joint.axis = VectorURDFToEigen(e.second->axis);

        switch (e.second->type) {
        case ::urdf::Joint::UNKNOWN:
            fprintf(stderr, "Unknown joint type\n");
            return false;
        case ::urdf::Joint::FIXED:
            joint.type = JointType::Fixed;
            break;
        case ::urdf::Joint::CONTINUOUS:
        case ::urdf::Joint::REVOLUTE:
            joint.type = JointType::Revolute;
            break;
        case ::urdf::Joint::PRISMATIC:
            joint.type = JointType::Prismatic;
            break;
        case ::urdf::Joint::PLANAR:
            joint.type = JointType::Planar;
            break;
        case ::urdf::Joint::FLOATING:
            joint.type = JointType::Floating;
            break;
        default:
            fprintf(stderr, "Unrecognized joint type\n");
            return false;
        }

        robot_model.joints.push_back(std::move(joint));
    }

    ////////////////////////////////////
    // Initialize variable properties //
    ////////////////////////////////////

    // ...for the world joint
    switch (wj.type) {
    case JointType::Fixed:
        break;
    case JointType::Planar:
        AddPlanarJointVariables(&wj.name, &robot_model.variables);
        break;
    case JointType::Floating:
        AddFloatingJointVariables(&wj.name, &robot_model.variables);
        break;
    default:
        fprintf(stderr, "World joint must be fixed, planar, or floating\n");
        return false;
    }

    // ...for the joints in the ::urdf
    for (auto& e : urdf->joints_) {
        switch (e.second->type) {
        case ::urdf::Joint::UNKNOWN:
            return false;
        case ::urdf::Joint::FIXED:
            break;
        case ::urdf::Joint::REVOLUTE:
        case ::urdf::Joint::PRISMATIC:
        {
            JointVariable v;
            v.name = e.first;
            if (e.second->limits) {
                v.limits.has_position_limits = true;
                v.limits.min_position = e.second->limits->lower;
                v.limits.max_position = e.second->limits->upper;
                v.limits.max_velocity = e.second->limits->velocity;
                v.limits.max_effort = e.second->limits->effort;
            } else {
                v.limits.has_position_limits = false;
                v.limits.min_position = -std::numeric_limits<double>::infinity();
                v.limits.max_position = std::numeric_limits<double>::infinity();
                v.limits.max_velocity = 0.0;
                v.limits.max_effort = 0.0;
            }
            robot_model.variables.push_back(std::move(v));
            break;
        }
        case ::urdf::Joint::CONTINUOUS:
        {
            JointVariable v;
            v.name = e.first;
            v.limits.has_position_limits = false;
            v.limits.min_position = -std::numeric_limits<double>::infinity();
            v.limits.max_position = std::numeric_limits<double>::infinity();
            if (e.second->limits) {
                v.limits.max_velocity = e.second->limits->velocity;
                v.limits.max_effort = e.second->limits->velocity;
            } else {
                v.limits.max_velocity = 0.0;
                v.limits.max_effort = 0.0;
            }
            robot_model.variables.push_back(std::move(v));
            break;
        }
        case ::urdf::Joint::PLANAR:
        {
            AddPlanarJointVariables(&e.first, &robot_model.variables);
            break;
        }
        case ::urdf::Joint::FLOATING:
        {
            AddFloatingJointVariables(&e.first, &robot_model.variables);
            break;
        }
        default:
            return false;
        }
    }

    ////////////////////////////////
    // Connect the kinematic tree //
    ////////////////////////////////

    auto get_joint = [](RobotModel* model, const std::string& name) -> Joint* {
        for (auto& joint : model->joints) {
            if (joint.name == name) {
                return &joint;
            }
        }
        return NULL;
    };

    // ...connect parent and child joints to links (child links stored within
    // each joint)
    for (auto& link : robot_model.links) {
        auto l = urdf->getLink(link.name);
        if (l->parent_joint) {
            link.parent = get_joint(&robot_model, l->parent_joint->name);
            if (link.parent == NULL) {
                fprintf(stderr, "Failed to find parent joint '%s'\n", l->parent_joint->name.c_str());
                return false;
            }
        }

        Joint* prev_joint = NULL;
        for (auto& cj : l->child_joints) {
            auto* child_joint = get_joint(&robot_model, cj->name);
            if (child_joint == NULL) {
                fprintf(stderr, "Failed to find child joint '%s'\n", cj->name.c_str());
                return false;
            }
            child_joint->sibling = prev_joint;
            prev_joint = child_joint;
        }
        link.children = prev_joint;
    }

    // ...connect parent and child links to joints
    for (auto& joint : robot_model.joints) {
        auto j = urdf->getJoint(joint.name);
        if (!j) continue; // skip the world joint

        joint.child = get_link(&robot_model, j->child_link_name);
        if (joint.child == NULL) {
            fprintf(stderr, "Failed to find child link '%s'\n", j->child_link_name.c_str());
            return false;
        }

        joint.parent = get_link(&robot_model, j->parent_link_name);
        if (joint.parent == NULL) {
            fprintf(stderr, "Failed to find parent link '%s'\n", j->parent_link_name.c_str());
            return false;
        }
    }

    // ...connect joints to variables
    JointVariable* vit = robot_model.variables.data();
    for (auto& joint : robot_model.joints) {
        joint.vfirst = vit;

        JointVariable* vtmp = NULL;
        switch (joint.type) {
        case JointType::Fixed:
            vtmp = vit;
            break;
        case JointType::Revolute: // fallthrough
        case JointType::Prismatic:
            vtmp = vit + 1;
            break;
        case JointType::Planar:
            vtmp = vit + 3;
            break;
        case JointType::Floating:
            vtmp = vit + 7;
            break;
        }

        for (auto* v = vit; v != vtmp; ++v) {
            v->joint = &joint;
        }

        vit = vtmp;
        joint.vlast = vit;
    }

    // ...set the root link
    robot_model.root_link = get_link(&robot_model, urdf->root_link_->name);
    if (robot_model.root_link == NULL) {
        fprintf(stderr, "Failed to find root link '%s'\n", urdf->root_link_->name.c_str());
        return false;
    }

    // ...finally, connect the root joint to the root link
    robot_model.root_joint = get_joint(&robot_model, wj.name);
    robot_model.root_joint->child = robot_model.root_link;
    robot_model.root_link->parent = robot_model.root_joint;

    auto CommonAncestor = [](RobotModel* model, const Joint* a, const Joint* b)
    {
        // pretty inefficient common ancestor computation...walk up the tree
        // from joint a to the root and run a depth-first search from each
        // intermediate node to see whether there is a path to b.
        for (auto* start = a; start != NULL; start = start->parent->parent) {
            std::vector<const Joint*> q;
            q.push_back(start);
            while (!q.empty()) {
                auto* j = q.back();
                q.pop_back();
                if (j == b) {
                    return start;
                }

                for (auto* child = j->child->children; child != NULL; child = child->sibling) {
                    q.push_back(child);
                }
            }
        }
        return (const Joint*)NULL;
    };

    // precompute the map from pairs of joints to their
    robot_model.ancestor_map.resize(robot_model.joints.size() * robot_model.joints.size());
    for (auto& ji : robot_model.joints) {
        auto i = &ji - robot_model.joints.data();
        for (auto& jj : robot_model.joints) {
            auto j = &jj - robot_model.joints.data();
            auto* ancestor = CommonAncestor(&robot_model, &ji, &jj);
            robot_model.ancestor_map[i * robot_model.joints.size() + j] = ancestor;
        }
    }

    *out = std::move(robot_model);
    return true;
}

auto GetName(const RobotModel* model) -> const std::string*
{
    return &model->name;
}

auto GetRootJoint(const RobotModel* model) -> const Joint*
{
    return model->root_joint;
}

auto GetRootLink(const RobotModel* model) -> const Link*
{
    return model->root_link;
};

auto GetLinkCount(const RobotModel* model) -> size_t
{
    return model->links.size();
}

auto GetJointCount(const RobotModel* model) -> size_t
{
    return model->joints.size();
}

auto GetVariableCount(const RobotModel* model) -> size_t
{
    return model->variables.size();
}

auto GetCollisionBodyCount(const RobotModel* model) -> size_t
{
    return model->collisions.size();
}

auto GetVisualBodyCount(const RobotModel* model) -> size_t
{
    return model->visuals.size();
}

auto GetLink(const RobotModel* model, int index) -> const Link*
{
    return &model->links[index];
}

auto GetJoint(const RobotModel* model, int index) -> const Joint*
{
    return &model->joints[index];
}

auto GetVariable(const RobotModel* model, int index) -> const JointVariable*
{
    return &model->variables[index];
}

auto GetCollisionBody(const RobotModel* model, int index) -> const LinkCollision*
{
    return &model->collisions[index];
}

auto GetVisualBody(const RobotModel* model, int index) -> const LinkVisual*
{
    return &model->visuals[index];
}

auto Links(const RobotModel* model) -> range<const Link*>
{
    return make_range(
            model->links.data(),
            model->links.data() + model->links.size());
}

auto Joints(const RobotModel* model) -> range<const Joint*>
{
    return make_range(
            model->joints.data(),
            model->joints.data() + model->joints.size());
}

auto Variables(const RobotModel* model) -> range<const JointVariable*>
{
    return make_range(
            model->variables.data(),
            model->variables.data() + model->variables.size());
}

auto CollisionBodies(const RobotModel* model) -> range<const LinkCollision*>
{
    return make_range(
            model->collisions.data(),
            model->collisions.data() + model->collisions.size());
}

auto VisualBodies(const RobotModel* model) -> range<const LinkVisual*>
{
    return make_range(
            model->visuals.data(),
            model->visuals.data() + model->visuals.size());
}

auto GetLinkIndex(const RobotModel* model, const Link* link) -> size_t
{
    return (size_t)(link - model->links.data());
}

auto GetJointIndex(const RobotModel* model, const Joint* joint) -> size_t
{
    return (size_t)(joint - model->joints.data());
}

auto GetVariableIndex(const RobotModel* model, const JointVariable* variable) -> size_t
{
    return (size_t)(variable - model->variables.data());
}

auto GetCollisionBodyIndex(const RobotModel* model, const LinkCollision* collision) -> size_t
{
    return (size_t)(collision - model->collisions.data());
}

auto GetVisualBodyIndex(const RobotModel* model, const LinkVisual* visual) -> size_t
{
    return (size_t)(visual - model->visuals.data());
}

auto GetName(const Link* link) -> const std::string*
{
    return &link->name;
}

auto GetName(const Joint* joint) -> const std::string*
{
    return &joint->name;
}

auto GetName(const JointVariable* variable) -> const std::string*
{
    return &variable->name;
}

auto GetLinkName(const RobotModel* model, int index) -> const std::string*
{
    return GetName(&model->links[index]);
}

auto GetJointName(const RobotModel* model, int index) -> const std::string*
{
    return GetName(&model->joints[index]);
}

auto GetVariableName(const RobotModel* model, int index) -> const std::string*
{
    return GetName(&model->variables[index]);
}

auto GetLink(const RobotModel* model, const char* name) -> const Link*
{
    for (auto& link : model->links) {
        if (link.name == name) {
            return &link;
        }
    }
    return NULL;
}

auto GetJoint(const RobotModel* model, const char* name) -> const Joint*
{
    for (auto& joint : model->joints) {
        if (joint.name == name) {
            return &joint;
        }
    }
    return NULL;
}

auto GetVariable(const RobotModel* model, const char* name) -> const JointVariable*
{
    for (auto& variable : model->variables) {
        if (variable.name == name) {
            return &variable;
        }
    }
    return NULL;
}

auto GetLink(const RobotModel* model, const std::string* name) -> const Link*
{
    for (auto& link : model->links) {
        if (link.name == *name) {
            return &link;
        }
    }
    return NULL;
}

auto GetJoint(const RobotModel* model, const std::string* name) -> const Joint*
{
    for (auto& joint : model->joints) {
        if (joint.name == *name) {
            return &joint;
        }
    }
    return NULL;
}

auto GetVariable(const RobotModel* model, const std::string* name)
    -> const JointVariable*
{
    for (auto& variable : model->variables) {
        if (variable.name == *name) {
            return &variable;
        }
    }
    return NULL;
}

auto VisualBodies(const Link* link) -> range<const LinkVisual*>
{
    // TODO: implicit conversion to const iterators
    return make_range<const LinkVisual*>(
            link->visual.p.first, link->visual.p.second);
}

auto CollisionBodies(const Link* link) -> range<const LinkCollision*>
{
    // TODO: see note above
    return make_range<const LinkCollision*>(
            link->collision.p.first, link->collision.p.second);
}

auto GetVariableCount(const Joint* joint) -> size_t
{
    return joint->vlast - joint->vfirst;
}

auto GetFirstVariable(const Joint* joint) -> const JointVariable*
{
    return joint->vfirst;
}

auto Variables(const Joint* joint) -> range<const JointVariable*>
{
    return make_range<const JointVariable*>(joint->vfirst, joint->vlast);
}

auto GetCommonRoot(
    const RobotModel* model,
    const Joint* a,
    const Joint* b)
    -> const Joint*
{
    auto i = GetJointIndex(model, a);
    auto j = GetJointIndex(model, b);
    return model->ancestor_map[i * GetJointCount(model) + j];
}

bool IsAncestor(const RobotModel* model, const Joint* a, const Joint* b)
{
    return GetCommonRoot(model, a, b) == a;
}

auto GetJointOfVariable(const JointVariable* variable) -> const Joint*
{
    return variable->joint;
}

auto GetVariableLimits(const JointVariable* variable) -> const VariableLimits*
{
    return &variable->limits;
}

auto GetDefaultPosition(const RobotModel* model, const JointVariable* variable)
    -> double
{
    switch (variable->joint->type) {
    case JointType::Fixed:
        assert(0);
        return std::numeric_limits<double>::quiet_NaN();
    case JointType::Revolute:
    case JointType::Prismatic:
        // NOTE: 0 should be fine limitless variables
        if (variable->limits.min_position <= 0.0 & 0.0 <= variable->limits.max_position) {
            return 0.0;
        } else {
            return 0.5 * (variable->limits.min_position + variable->limits.max_position);
        }
    case JointType::Planar:
        return 0.0;
    case JointType::Floating:
        // 1 for quaternion w, 0 everything else
        if (variable - variable->joint->vfirst == 6) { // w in the 7th position
            return 1.0;
        } else {
            return 0.0;
        }
    }
}

} // namespace urdf
} // namespace motion
} // namespace smpl

