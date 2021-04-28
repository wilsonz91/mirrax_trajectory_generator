cmake_minimum_required(VERSION 2.8.3)
project(mirrax_trajectory_generator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
# cs_add_library(${PROJECT_NAME}
#   src/polynomial_6D.cc
# )
# target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(main
        src/polynomial.cc
        src/kinematics.cpp
        src/polynomial_node.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
