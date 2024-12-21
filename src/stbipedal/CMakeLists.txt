cmake_minimum_required(VERSION 3.5)
project(stbipedal)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Debug -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ambot_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)



# include external project of cybergear sdk
include(ExternalProject)
ExternalProject_Add(
    cybergear_sdk
    GIT_REPOSITORY https://github.com/sunzhon/cybergear_motor_sdk.git
    GIT_TAG master  # 可选择分支或特定版本
    PREFIX ${CMAKE_BINARY_DIR}/_deps
    SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/src/cybergear_sdk
    BINARY_DIR ${CMAKE_BINARY_DIR}/_deps/src/cybergear_sdk-build
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
    #CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    #INSTALL_COMMAND ""  # Disable the install command
    UPDATE_COMMAND "" #prevent update step
)

set(CYBERGEAR_ROOT ${CMAKE_BINARY_DIR}/_deps/src/cybergear_sdk)
set(CYBERGEAR_BUILD ${CYBERGEAR_ROOT}-build)
message("CYBERGEAR_BUILD: ${CYBERGEAR_BUILD}")


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include ${colcon_INCLUDE_DIRS}   ${Boost_INCLUDE_DIRS})

# include unitree motor
set(LIB_PATH "${CMAKE_SOURCE_DIR}/lib")
link_directories(${LIB_PATH})
message("lib: ${LIB_PATH}")

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
	set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(ENV{LD_LIBRARY_PATH} "$ENV{LD_LIBRARY_PATH}:${LIB_PATH}")
message(" $ENV{LD_LIBRARY_PATH}")


add_executable(${PROJECT_NAME}_node src/main.cpp src/node.cpp src/yesense.cpp src/analysis_data.cpp)

target_link_libraries(${PROJECT_NAME}_node "${LIB_PATH}/${EXTRA_LIBS}" ${Boost_LIBRARIES} Boost::thread Boost::system "${CYBERGEAR_BUILD}/install/lib/libcybergear_sdk.so")

target_include_directories(${PROJECT_NAME}_node PRIVATE ${CYBERGEAR_BUILD}/install/include/include)

# 在你的目标编译时，确保这个外部库已编译完成
add_dependencies(${PROJECT_NAME}_node cybergear_sdk)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs ambot_msgs serial )

ament_export_dependencies(Boost)
ament_export_libraries(${PROJECT_NAME} Boost::thread Boost::system)

add_subdirectory(thirdparty/pybind11)
pybind11_add_module(unitree_actuator_sdk thirdparty/python_wrapper/wrapper.cpp)


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


install(FILES ${LIB_PATH}/${EXTRA_LIBS}
        DESTINATION lib)

install(FILES  "${CYBERGEAR_BUILD}/install/lib/libcybergear_sdk.so"
        DESTINATION lib)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


