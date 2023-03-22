cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME FCL_TEST)

set(CMAKE_CXX_STANDARD 17)

project(${PROJECT_NAME})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)





set(FCL_TEST_SOURCE_DIR
        src/sources
        )

set(FCL_TEST_INCLUDE_DIR
        src/include
        )

set(FCL_TEST_SOURCE_FILES
        ${FCL_TEST_SOURCE_DIR}/MpcControl.cpp
        ${FCL_TEST_SOURCE_DIR}/traj.cpp
        ${FCL_TEST_SOURCE_DIR}/MPC_Optimal.cpp
        ${FCL_TEST_SOURCE_DIR}/admittance_controller.cpp        
        ${FCL_TEST_SOURCE_DIR}/IK_qp.cpp   
       # ${FCL_TEST_SOURCE_DIR}/RRTalgorithm/rrtstar.cpp
       # ${FCL_TEST_SOURCE_DIR}/RRTalgorithm/depth_based_rrt_star.cpp
            )
set(FCL_TEST_HEADER_FILES
    #/home/oem/catkin_ws/qpOASES/include/qpOASES/*.hpp
    #/home/oem/catkin_ws/qpOASES/include/qpOASES/extras/*.hpp
    ${FCL_TEST_INCLUDE_DIR}/traj.h
    ${FCL_TEST_INCLUDE_DIR}/MPC_Optimal.h
    ${FCL_TEST_INCLUDE_DIR}/admittance_controller.h
    ${FCL_TEST_INCLUDE_DIR}/IK_qp.h
    # ${FCL_TEST_INCLUDE_DIR}/rrtstar.h
            )

#find_path(FCL_INCLUDE_DIRS
#NAMES fcl/collision.h
#HINTS ${PC_FCL_INCLUDEDIR}
#PATHS "${CMAKE_INSTALL_PREFIX}/include")

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
set(TRAJ_GENERATOR_INCLUDE_DIR /home/mlei/catkin_ws/traj_generate/src/src/include)
set(TRAJ_GENERATOR_LIBRARIES /home/mlei/catkin_ws/traj_generate/devel/lib/libTRAJ_GENERATOR_lib.so)

set(arm_collision_detection_INCLUDE_DIR /home/mlei/catkin_ws/src/mpc/fcl_test/src/include)
set(arm_collision_detection_LIBRARIES /home/mlei/catkin_ws/devel/lib/libarm_collision_detection.so)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(qpOASES REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(PythonLibs REQUIRED) 
find_package(pinocchio)







find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
  urdf
  kdl_parser
  TRAJ_GENERATOR
  arm_collision_detection
  fcl
  octomap
  gazebo_msgs
  tf
)

add_service_files(
  FILES
  state_service.srv
)


add_message_files(
  FILES
  state.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fcl_test
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime gazebo_msgs tf
DEPENDS Eigen3 qpOASES fcl TRAJ_GENERATOR ccd
)





###########
## Build ##
###########

include_directories(${cartesian_interface_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TRAJ_GENERATOR_INCLUDE_DIR})

include_directories(${PINOCCHIO_INCLUDE_DIRS})

include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${FCL_TEST_INCLUDE_DIR}
  ${QPOASES_INLUDE_DIRS}
  ${fcl_INCLUDE_DIR}
  ${TRAJ_GENERATOR_INCLUDE_DIR}
  ${arm_collision_detection_DIR} 
)
## link_directories(/home/oem/mosek/8/tools/platform/linux64x86/src/fusion_cxx)


## add_library(ik_solve ${FCL_TEST_SOURCE_FILES} ${FCL_TEST_HEADER_FILES})


add_library(FCL_TEST_lib ${FCL_TEST_SOURCE_FILES} ${FCL_TEST_HEADER_FILES})
add_dependencies(FCL_TEST_lib ${catkin_EXPORTED_TARGETS})


target_link_libraries(FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} ${arm_collision_detection_LIBRARIES} ${CCD_LIBRARIES} ${YAMLCPP_LIBRARIES} fcl XBotInterface::XBotInterface ${octomap_LIBRARIES} ${cartesian_interface_LIBRARIES})



#add_executable(MPC_modular src/sources/MpcControl.cpp) 
#add_dependencies(MPC_modular ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MPC_modular FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES)

#add_executable(MPC_Center src/sources/MpcControl_Center.cpp) 
#add_dependencies(MPC_Center ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MPC_Center FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
#${cartesian_interface_LIBRARIES})


#add_executable(MPC_Center_sim src/sources/MpcControl_Center.cpp) 
#add_dependencies(MPC_Center_sim ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MPC_Center_sim FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
#${cartesian_interface_LIBRARIES})



add_executable(IK_service src/traning_MPC/IK_service.cpp) 
add_dependencies(IK_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(IK_service FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})



add_executable(MpcControl_HiMPC src/sources/MpcControl_HiMPC.cpp) 
add_dependencies(MpcControl_HiMPC ${catkin_EXPORTED_TARGETS})
target_link_libraries(MpcControl_HiMPC FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})


#add_executable(TF_Pub_End src/sources/TF_Pub_End.cpp) 
#add_dependencies(TF_Pub_End ${catkin_EXPORTED_TARGETS})
#target_link_libraries(TF_Pub_End FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES mosek64)


#add_executable(LeastSquare src/sources/LeastSquare.cpp) 
#add_dependencies(LeastSquare ${catkin_EXPORTED_TARGETS})
#target_link_libraries(LeastSquare FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES mosek64)

#add_executable(MpcControl_CenterTest src/sources/MpcControl_CenterTest.cpp) 
#add_dependencies(MpcControl_CenterTest ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MpcControl_CenterTest FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES mosek64)

#add_executable(Output_GroundTruth src/sources/Output_GroundTruth.cpp) 
#add_dependencies(Output_GroundTruth ${catkin_EXPORTED_TARGETS})
#target_link_libraries(Output_GroundTruth FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES mosek64)

#add_executable(MPC_HiModular src/sources/MPC_HiModular.cpp) 
#add_dependencies(MPC_HiModular ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MPC_HiModular FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
#${cartesian_interface_LIBRARIES})


# add_executable(training_MPCIK src/traning_MPC/training_MPCIK.cpp) 
# add_dependencies(training_MPCIK ${catkin_EXPORTED_TARGETS})
# target_link_libraries(training_MPCIK FCL_TEST_lib ${catkin_LIBRARIES} ${fcl_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
# ${cartesian_interface_LIBRARIES})

add_executable(IKtest src/traning_MPC/IKtest.cpp) 
add_dependencies(IKtest ${catkin_EXPORTED_TARGETS})
target_link_libraries(IKtest FCL_TEST_lib ${catkin_LIBRARIES} ${fcl_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})



# add_executable(IK_compare_sing_ag src/sources/IK_compare_sing_ag.cpp) 
# add_dependencies(IK_compare_sing_ag ${catkin_EXPORTED_TARGETS})
# target_link_libraries(IK_compare_sing_ag FCL_TEST_lib ${catkin_LIBRARIES} ${fcl_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
# ${cartesian_interface_LIBRARIES})

# add_executable(IK_compare_sing_ag2 src/sources/IK_compare_sing_ag2.cpp) 
# add_dependencies(IK_compare_sing_ag2 ${catkin_EXPORTED_TARGETS})
# target_link_libraries(IK_compare_sing_ag2 FCL_TEST_lib ${catkin_LIBRARIES} ${fcl_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
# ${cartesian_interface_LIBRARIES})


add_executable(IK_compare_sing src/sources/IK_compare_sing.cpp) 
add_dependencies(IK_compare_sing ${catkin_EXPORTED_TARGETS})
target_link_libraries(IK_compare_sing FCL_TEST_lib ${catkin_LIBRARIES} ${fcl_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})

add_executable(MPC_HiModular_Experiment2 src/sources/Hi_experiment/MPC_HiModular_Experiment2.cpp) 
add_dependencies(MPC_HiModular_Experiment2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(MPC_HiModular_Experiment2 FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})

add_executable(admittance_controller_MPC src/sources/Hi_experiment/admittance_controller_MPC.cpp) 
add_dependencies(admittance_controller_MPC ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_controller_MPC FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})


#add_executable(onlinereplan src/sources/multiobjectcollision/onlinereplan.cpp) 
#add_dependencies(onlinereplan ${catkin_EXPORTED_TARGETS})
#target_link_libraries(onlinereplan FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
#${cartesian_interface_LIBRARIES})

#add_executable(MpcControl_Compare src/sources/MpcControl_Compare.cpp) 
#add_dependencies(MpcControl_Compare ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MpcControl_Compare FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
#${cartesian_interface_LIBRARIES})

#add_executable(MPCMODULAR_TEST src/sources/MPCMODULAR_TEST.cpp) 
#add_dependencies(MPCMODULAR_TEST ${catkin_EXPORTED_TARGETS})
#target_link_libraries(MPCMODULAR_TEST FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
#${cartesian_interface_LIBRARIES})

add_executable(MPConlineVerfication src/sources/MPConlineVerfication.cpp) 
add_dependencies(MPConlineVerfication ${catkin_EXPORTED_TARGETS})
target_link_libraries(MPConlineVerfication FCL_TEST_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${fcl_LIBRARIES} ${fcl_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARY_DIRS} ${TRAJ_GENERATOR_LIBRARIES} qpOASES 
${cartesian_interface_LIBRARIES})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fcl_test.cpp
# )


