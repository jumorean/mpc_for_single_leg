cmake_minimum_required(VERSION 3.0.2)
project(single_leg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/configDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/configDir.h)
configure_file(${CMAKE_SOURCE_DIR}/include/configDir.h.in ${CMAKE_SOURCE_DIR}/include/configDir.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


set(target_include_dirs "")
set(target_libs "")


find_package(catkin REQUIRED COMPONENTS
  ct_core
  ct_models
  ct_optcon
  ct_rbd
  roscpp
)

catkin_package(

)

set(target_src
        src/motor.cpp
        src/joint.cpp
        src/transform.cpp
        src/io.cpp
        )

include_directories(
  include/single_leg/
  ${catkin_INCLUDE_DIRS}
)

add_executable(single_leg_test src/single_leg.cpp)
target_link_libraries(single_leg_test ct_core ct_optcon ${catkin_LIBRARIES})
add_dependencies(single_leg_test ${catkin_EXPORTED_TARGETS})


add_executable(mpc_test src/mpc_test.cpp)
target_link_libraries(mpc_test ct_core ct_optcon)


add_executable(mpc.exe  src/elmo.cpp ${target_src} )
target_link_libraries(mpc.exe ct_core ct_optcon soem ${catkin_LIBRARIES})
target_include_directories(mpc.exe PUBLIC /usr/local/include/soem)
add_dependencies(mpc.exe ${catkin_EXPORTED_TARGETS})

