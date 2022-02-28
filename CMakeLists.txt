cmake_minimum_required(VERSION 3.20)
project(Planning)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
        include
        SYSTEM
        third_party
        ${Eigen3_INCLUDE_DIRS}
)



set(CMAKE_CXX_STANDARD 14)

#set(Sources a_star.cpp)
#set(Headers a_star.h)
#add_executable(Planning ${Sources} ${Headers})

add_executable(Planning main.cpp a_star.cpp a_star.h)

target_include_directories(${PROJECT_NAME} PRIVATE "/usr/include/python3.8")
target_link_libraries(${PROJECT_NAME} python3.8)