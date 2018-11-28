cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

project(OsqpFeasibility)

find_package(OsqpEigen)
find_package(EigenMatio CONFIG REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

#MPCExample
add_executable(OsqpFeasibility OsqpFeasibility.cpp)
target_link_libraries(OsqpFeasibility
  OsqpEigen::OsqpEigen
  osqp::osqp
  EigenMatio
  "-lmatio")
