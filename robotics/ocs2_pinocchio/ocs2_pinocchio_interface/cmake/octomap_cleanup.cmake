# Strip octomap from hpp-fcl::hpp-fcl / coal::coal INTERFACE_LINK_LIBRARIES.
#
# Why: pinocchio's collision backends (hpp-fcl on Jazzy, coal on Lyrical+) are
# shipped as prebuilt shared libraries whose DT_NEEDED entries bake in a
# specific octomap SONAME (e.g. liboctomap.so.1.9 on Jazzy). Their *Config.cmake
# additionally does find_dependency(octomap), which on multi-octomap systems
# may resolve to a different SONAME (e.g. liboctomap.so.1.10 from
# ros-jazzy-octomap). The resulting hpp-fcl::hpp-fcl imported target then
# advertises an octomap whose SONAME mismatches the one its own SO already
# DT_NEEDEDs. Linking against that target drags BOTH octomap SONAMEs into
# the binary, so the dynamic linker maps two distinct copies of the octomap
# C++ classes into one process -- which manifests as
# "double free or corruption (fasttop)" on global tear-down.
#
# OCS2 never references octomap symbols directly; we only get them transitively
# through the collision backend. Dropping octomap from the backend target's
# INTERFACE_LINK_LIBRARIES lets `ld --as-needed` strip octomap entirely from
# the OCS2 binary's DT_NEEDED list. octomap then enters the process exactly
# once via the DT_NEEDED entry baked into libhpp-fcl.so / libcoal.so itself,
# guaranteeing ABI consistency regardless of which distro or octomap version
# is installed.
#
# Hooked via ament_package(CONFIG_EXTRAS_POST ...), which runs AFTER
# ament_cmake_export_dependencies-extras (i.e. after find_dependency(pinocchio)
# has materialised the hpp-fcl / coal imported targets).
foreach(_ocs2_collision_target hpp-fcl::hpp-fcl coal::coal)
  if(TARGET ${_ocs2_collision_target})
    get_target_property(_ocs2_link_libs ${_ocs2_collision_target} INTERFACE_LINK_LIBRARIES)
    if(_ocs2_link_libs)
      list(REMOVE_ITEM _ocs2_link_libs octomap octomath)
      set_target_properties(${_ocs2_collision_target} PROPERTIES
        INTERFACE_LINK_LIBRARIES "${_ocs2_link_libs}")
    endif()
  endif()
endforeach()
unset(_ocs2_collision_target)
unset(_ocs2_link_libs)
