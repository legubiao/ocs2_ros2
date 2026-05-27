/******************************************************************************
Copyright (c) OCS2 contributors. All rights reserved.

Provides a unified ocs2::collision:: alias that points at whichever upstream
collision library pinocchio is actually linked against on the current
toolchain:
  - pinocchio >= 3 with coal (Ubuntu 26 / ROS Lyrical+): ocs2::collision == coal
  - legacy pinocchio with hpp-fcl (Jazzy and earlier):    ocs2::collision == hpp::fcl

Includes the relevant upstream headers and exposes a stable typedef for the
homogeneous transform type, whose name changed from Transform3f (hpp-fcl) to
Transform3s (coal).

Pinocchio publishes the discriminator via INTERFACE_COMPILE_DEFINITIONS
(PINOCCHIO_WITH_COAL or PINOCCHIO_WITH_HPP_FCL), so consumers that link
ocs2_pinocchio_interface get the right branch automatically. As a last-resort
fallback (e.g. when this header is parsed before any pinocchio header is
visible), the __has_include probe picks whichever upstream is available.
******************************************************************************/

#pragma once

#include <pinocchio/fwd.hpp>

#if defined(PINOCCHIO_WITH_COAL) || \
    (__has_include(<coal/collision_object.h>) && !__has_include(<hpp/fcl/collision_object.h>))

#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>

namespace ocs2 {
namespace collision = ::coal;
using collision_transform_t = ::coal::Transform3s;
}  // namespace ocs2

#elif defined(PINOCCHIO_WITH_HPP_FCL) || __has_include(<hpp/fcl/collision_object.h>)

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace ocs2 {
namespace collision = ::hpp::fcl;
using collision_transform_t = ::hpp::fcl::Transform3f;
}  // namespace ocs2

#else
#error "Neither <coal/collision_object.h> nor <hpp/fcl/collision_object.h> found. \
Install ros-<distro>-coal or ros-<distro>-hpp-fcl."
#endif
