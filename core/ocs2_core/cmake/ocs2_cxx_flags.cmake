# OCS2 project-wide compile settings and dependency resolution helper.
#
# Exported via ament_package(CONFIG_EXTRAS ...) so that downstream packages obtain
# these variables - and the Boost imported component targets referenced by
# ocs2_core::ocs2_core - automatically via find_package(ocs2_core).
#
# Ordering note: this file is included BEFORE
# ament_cmake_export_targets-extras.cmake, hence before any attempt to import the
# ocs2_core::ocs2_core target. This is intentional: the imported target references
# Boost imported targets in its INTERFACE_LINK_LIBRARIES, all of which must be
# defined when the export file is evaluated. ament_export_dependencies(Boost)
# cannot do this because it calls find_package(Boost) without COMPONENTS, so the
# component imported targets are not created.
#
# Additional flags can be appended at configure time, e.g.:
#   colcon build --cmake-args -DOCS2_CXX_FLAGS=-march=native\;-mtune=native

# Resolve the Boost components used by ocs2_core in its public interface.
# CMP0167 (CMake 4+) removes the FindBoost shim; opt in so this works on Ubuntu
# 26 / Boost 1.90 modular packaging while remaining a no-op on older CMake.
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost CONFIG REQUIRED COMPONENTS
        log
        log_setup
        filesystem
)

# Boost.System has been header-only since 1.69 and was dropped as a separately
# packaged component on Ubuntu 26 / Boost 1.90 (the Boost::system target no
# longer exists). Probe for it; consumers should link OCS2_BOOST_SYSTEM_TARGET
# instead of Boost::system directly.
find_package(Boost CONFIG QUIET COMPONENTS system)
if(TARGET Boost::system)
    set(OCS2_BOOST_SYSTEM_TARGET Boost::system)
else()
    set(OCS2_BOOST_SYSTEM_TARGET Boost::headers)
endif()

# Public compile flags propagated to consumers of any OCS2 library target.
#   -pthread / -fopenmp are intentionally omitted: they are provided transitively
#   by Threads::Threads and OpenMP::OpenMP_CXX imported targets.
#   -fPIC is listed here (rather than relying on the POSITION_INDEPENDENT_CODE
#   target property) so that every OCS2 static library that consumes these flags
#   produces PIC code; this is required because downstream SHARED libraries (e.g.
#   ocs2_pinocchio_interface) link them in.
list(APPEND OCS2_CXX_FLAGS
    -Wfatal-errors
    -fPIC
)

# Public compile definitions propagated to consumers.
list(APPEND OCS2_CXX_DEFINITIONS
    BOOST_ALL_DYN_LINK
)

# Private link options applied only to ocs2_core itself (not propagated to consumers).
# --no-as-needed is needed at ocs2_core's link step to keep Boost.Log linkage alive;
# consumers do not need it because they reference symbols directly.
list(APPEND OCS2_LINK_FLAGS_PRIVATE
    -Wl,--no-as-needed
)
