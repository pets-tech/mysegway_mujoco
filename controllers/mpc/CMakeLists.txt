cmake_minimum_required(VERSION 3.10)
project(mpc)

set(ZEROMQ_LIBRARY "/usr/lib/x86_64-linux-gnu/libzmq.so")

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_executable(mpc mpc.cpp)

target_include_directories(mpc PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIRS}
)

target_link_libraries(mpc 
    OsqpEigen::OsqpEigen
    ${ZEROMQ_LIBRARY}
)

find_package(Threads REQUIRED)
target_link_libraries(mpc Threads::Threads)