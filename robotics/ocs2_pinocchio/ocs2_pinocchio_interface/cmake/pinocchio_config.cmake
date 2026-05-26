# Add pinocchio flags
# Collision support flags (e.g. PINOCCHIO_WITH_HPP_FCL / PINOCCHIO_WITH_COAL)
# are no longer set here; pinocchio::pinocchio publishes them via its imported
# target's INTERFACE_COMPILE_DEFINITIONS, which works for both legacy hpp-fcl
# and the newer coal-based pinocchio.
set(OCS2_PINOCCHIO_FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -Wno-psabi                            # to silence GCC ABI notes on ARM/Jetson
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)