//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
#define SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H

#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/util/system.hh>
#include <gazebo/ode/common.h>
#include <ignition/math/Vector3.hh>

namespace gazebo {
namespace physics {

class GZ_PHYSICS_VISIBLE LivoxOdeMultiRayShape : public MultiRayShape {
 public:
    /// \brief Constructor.
    /// \param[in] _parent Parent Collision.
    explicit LivoxOdeMultiRayShape(CollisionPtr _parent);

    /// \brief Destructor.
    virtual ~LivoxOdeMultiRayShape();

    // Documentation inherited.
    virtual void UpdateRays();

    virtual void Init();

    std::vector<RayShapePtr>& RayShapes() { return rays; }

    /// \brief Add a ray to the collision.
    /// \param[in] _start Start of a ray.
    /// \param[in] _end End of a ray.
    void AddRay(const ignition::math::Vector3d &_start,
                const ignition::math::Vector3d &_end);

   
   //  GZ_REGISTER_MODEL_PLUGIN(LivoxOdeMultiRayShape)

 private:
    /// \brief Ray-intersection callback.
    /// \param[in] _data Pointer to user data.
    /// \param[in] _o1 First geom to check for collisions.
    /// \param[in] _o2 Second geom to check for collisions.
    static void UpdateCallback(void *_data, dGeomID _o1, dGeomID _o2);

    /// \brief Space to contain the ray space, for efficiency.
    dSpaceID superSpaceId;

    /// \brief Ray space for collision detector.
    dSpaceID raySpaceId;

    std::vector<RayShapePtr> livoxRays;
};

}  // namespace physics
}  // namespace gazebo

#endif  // SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
