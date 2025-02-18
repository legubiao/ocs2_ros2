# The list of compiler flags used in ocs2 can be prefixed with ament_cmake config
# Addition flags are to be separated by \;
# For example, to turn on architecture specific optimizations:
#   ament_cmake config --cmake-args -DOCS2_CXX_FLAGS=-march=native\;-mtune=native
list(APPEND OCS2_CXX_FLAGS
        "-pthread"
        "-Wfatal-errors"
        "-Wl,--no-as-needed"
        "-fPIC"
)

# Force Boost dynamic linking
list(APPEND OCS2_CXX_FLAGS
        "-DBOOST_ALL_DYN_LINK"
)

# Add OpenMP flags
if (NOT DEFINED OpenMP_CXX_FOUND)
    find_package(OpenMP REQUIRED)
endif (NOT DEFINED OpenMP_CXX_FOUND)
list(APPEND OCS2_CXX_FLAGS
        ${OpenMP_CXX_FLAGS}
)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)