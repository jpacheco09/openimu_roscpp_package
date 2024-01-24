cmake_minimum_required(VERSION 3.10)
project(openimu_roscpp_package)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_C_COMPILER "/usr/lib/ccache/clang-10")
#set(CMAKE_CXX_COMPILER "/usr/lib/ccache/clang++-10")

if (MSVC)
    # warning level 4
    add_compile_options(/W4)
else()
    # additional warnings
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# libserial pthreads linking
set(CMAKE_MODULE_PATH $(PROJECT_SOURCE_DIR)/cmake)
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED) 
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs tf geometry_msgs)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openimu_attitude_publisher
#  CATKIN_DEPENDS libserial roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/openimu_attitude_publisher_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_include_directories(${PROJECT_NAME}_node PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})


# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
