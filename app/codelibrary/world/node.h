 //
// Copyright 2020-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_NODE_H_
#define CODELIBRARY_WORLD_NODE_H_

#include <any>
#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "codelibrary/world/camera/frustum.h"
#include "codelibrary/world/kernel/collidable.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/kernel/renderable.h"
#include "codelibrary/world/kernel/transformable.h"

namespace cl {
namespace world {

/**
 * World node.
 */
class Node : public world::Transformable,
             public world::Renderable,
             public world::Collidable {
    using Children = std::unordered_set<Node*>;
    using Transform = gl::RTransform;

    friend class Scene;

public:
    Node() = default;

    explicit Node(const std::string& name)
        : name_(name) {}

    Node(const Node& node) = delete;

    Node(const std::string& name, RenderObject* object)
        : Renderable(object), name_(name) {}

    explicit Node(RenderObject* object)
        : Renderable(object) {}

    virtual ~Node() {
        if (parent_) {
            size_t n_erased = parent_->children_.erase(this);
            CHECK(n_erased == 1);
            parent_ = nullptr;
        }
        for (auto c : children_) {
            c->parent_ = nullptr;
        }
    }

    Node& operator=(const Node&) = delete;

    /**
     * Add an existing node as the child of this node.
     *
     * Require that the adding node is an isolate node.
     */
    void AddNode(Node* node) {
        CHECK(!node->parent_) << "Cannot be added a node repeatedly.";
        children_.insert(node);
        node->parent_ = this;
    }

    /**
     * Get the child node according to the given name.
     *
     * If there are several nodes have the same name, return the first one in
     * unordered_set.
     *
     * Note that this code has a time cost of O(N). Should be override in
     * special cases.
     */
    virtual Node* GetNode(const std::string& name) const {
        for (Node* node : children_) {
            if (node->name() == name) return node;
        }
        return nullptr;
    }

    /**
     * Erase a node.
     *
     * Require that the erasing node is a child of this node. Note that it dose
     * not destroy the node.
     */
    void EraseNode(Node* node) {
        CHECK(node->parent_ == this);

        children_.erase(node);
        node->parent_ = nullptr;
    }

    /**
     * Clear all children nodes.
     */
    void ClearChildren() {
        for (auto node : children_) {
            node->ClearChildren();
            node->parent_ = nullptr;
        }
        children_.clear();
    }

    /**
     * Get the bounding box in local coordinates of this node.
     */
    virtual RBox3D GetBoundingBox() const {
        RBox3D box;
        for (RenderObject* o : render_list_) {
            // Need to convert FBox to RBox.
            FBox3D fbox = o->bounding_box();
            if (fbox.empty()) continue;

            RBox3D rbox(fbox.x_min(), fbox.x_max(), fbox.y_min(), fbox.y_max(),
                        fbox.z_min(), fbox.z_max());
            box.Union(rbox);
        }

        for (auto child : children_) {
            box.Union(child->transform_(child->GetBoundingBox()));
        }
        return box;
    }

    /**
     * Pick this node by ray.
     *
     * It only works when collision shapes is not empty.
     */
    virtual bool Pick(const RRay3D& ray, RPoint3D* point = nullptr) const {
        if (collision_shapes_.empty()) return false;

        Transform transform = this->global_transform_.Inverse();
        RRay3D r(transform(ray.origin()), transform(ray.direction()));

        double dis = DBL_MAX;
        for (const std::unique_ptr<CollisionShape>& shape : collision_shapes_) {
            RPoint3D p;
            if (shape.get()->Pick(r, &p)) {
                if (!point) return true;

                double d = Distance(r.origin(), p);
                if (d < dis) {
                    dis = d;
                    *point = p;
                }
            }
        }

        if (dis < DBL_MAX) {
            if (point) {
                // Need to convert the point to the world coordinate.
                *point = global_transform_(*point);
            }

            return true;
        }

        return false;
    }

    /**
     * Check if the node can be culled by frustum.
     */
    virtual bool Cull(const Frustum& frustum) const {
        RBox3D box = this->GetBoundingBox();
        if (box.empty()) return false;

        // Compute the vertices of node's bounding box.
        Array<RPoint3D> vertices = {
            {box.x_min(), box.y_min(), box.z_min()},
            {box.x_min(), box.y_min(), box.z_max()},
            {box.x_min(), box.y_max(), box.z_min()},
            {box.x_min(), box.y_max(), box.z_max()},
            {box.x_max(), box.y_min(), box.z_min()},
            {box.x_max(), box.y_min(), box.z_max()},
            {box.x_max(), box.y_max(), box.z_min()},
            {box.x_max(), box.y_max(), box.z_max()}
        };
        for (auto& v : vertices) {
            v = this->global_transform() * v;
        }

        return frustum.Cull(vertices);
    }

    /**
     * Check if the node can be select by a 3D box.
     */
    virtual bool Select(const RBox3D& box) {
        Frustum frustum(box);
        return !Cull(frustum);
    }

    /**
     * Check if this node is an instance node or not.
     */
    virtual bool is_instance() const {
        return false;
    }

    /**
     * Call when the node has been transformed before the rendering frame
     * begins.
     */
    virtual void Transformed() {}

    /**
     * The global transformation is automatically updated every frame. However,
     * users also have the option to update it manually using this function.
     */
    void UpdateGlobalTransform() {
        Array<Node*> nodes;
        Node* node = this;
        do {
            nodes.push_back(node);
            if (node == node->parent_) break;
            node = node->parent_;
        } while (node);
        std::reverse(nodes.begin(), nodes.end());

        global_transform_.SetIdentity();
        for (auto node : nodes) {
            global_transform_ = global_transform_ * node->GetTransform();
        }
    }

    /**
     * Return true if this node has 'node' as its child.
     */
    bool HasChild(Node* node) const {
        return children_.find(node) != children_.end();
    }

    /**
     * Return true if 'node' is the child of this node.
     */
    bool is_child(Node* node) const {
        return children_.find(node) != children_.end();
    }

    /**
     * Set user-defined property data.
     */
    template <typename T>
    void set_property(const std::string& name, const T& v) {
        static_assert(!std::is_pointer<std::decay_t<T>>::value,
                      "Type T must not be pointer");

        properties_[name] = v;
    }

    /**
     * Get user-defined property data.
     */
    template <typename T>
    T get_property(const std::string& name) const {
        auto it = properties_.find(name);
        if (it != properties_.end()) {
            return std::any_cast<T>(it->second);
        }

        return T();
    }

    // O(1) functions to access the members.
    const std::string& name()              const { return name_;              }
    Node* parent()                               { return parent_;            }
    const Node* parent()                   const { return parent_;            }
    const Children& children()             const { return children_;          }
    bool has_child()                       const { return !children_.empty(); }
    void set_name(const std::string& name)       { name_ = name;              }
    bool fixed()                           const { return fixed_;             }
    void set_fixed(bool flag)                    { fixed_ = flag;             }
    void set_cast_shadow(bool flag)              { cast_shadow_ = flag;       }
    void set_receive_shadow(bool flag)           { receive_shadow_ = flag;    }
    bool cast_shadow()                     const { return cast_shadow_;       }
    bool receive_shadow()                  const { return receive_shadow_;    }
    bool is_checked()                      const { return is_checked_;        }
    void set_checked(bool flag)                  { is_checked_ = flag;        }

protected:
    // The name of the node.
    std::string name_;

    // Parent node.
    Node* parent_ = nullptr;

    // Children nodes.
    Children children_;

    // When a node is fixed its 'transform' is not used in scene rendering.
    bool fixed_ = false;

    // Cast shadow or not.
    bool cast_shadow_ = true;

    // Receive shadow or not.
    bool receive_shadow_ = true;

    // Check if the node is checked or not.
    bool is_checked_ = false;

    // Store user data.
    std::unordered_map<std::string, std::any> properties_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_NODE_H_
