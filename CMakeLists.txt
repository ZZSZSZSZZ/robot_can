cmake_minimum_required(VERSION 3.22)
project(robot_can VERSION 1.0.0)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

find_package(Threads REQUIRED)

# 定义库目标
add_library(robot_can
        src/common/error_code.cpp
        src/can/can_coding.cpp
        src/can/can_frame_router.cpp
        src/can/can_receiver.cpp
        src/can/can_socket_impl.cpp
        src/motor/motor_manager.cpp
        src/motor/motor_factory.cpp
        src/motor/drivers/eyou/eyou_units.cpp
        src/motor/drivers/eyou/eyou_command.cpp
        src/motor/drivers/eyou/eyou_specification.cpp
        src/motor/drivers/eyou/eyou_motor.cpp
        src/motor/drivers/eyou/eyou_factory.cpp
        src/robot/arm_component.cpp
        src/robot/robot.cpp
)
set_target_properties(robot_can PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(
        robot_can PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS robot_can EXPORT robot_can_export)
install(DIRECTORY include/ DESTINATION include)

# logger宏开关 (true 启用, false 禁用)
target_compile_definitions(robot_can PRIVATE LOG_ENABLED=true)
add_definitions(-DLOG_ENABLED=1)

# CMake package
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/RobotCAN)
install(
        EXPORT robot_can_export
        DESTINATION ${INSTALL_CMAKE_DIR}
        NAMESPACE RobotCAN::
        FILE RobotCANTargets.cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
        RobotCANConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/RobotCANConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/RobotCANVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/RobotCANConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/RobotCANVersion.cmake
        DESTINATION ${INSTALL_CMAKE_DIR})

# pkg-config package
if (IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
    set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else ()
    set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif ()
if (IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
    set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else ()
    set(PKG_CONFIG_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
endif ()
configure_file(robot-can.pc.in ${CMAKE_CURRENT_BINARY_DIR}/robot-can.pc
        @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/robot-can.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

target_link_libraries(robot_can)

# Add tests
if (BUILD_TESTING)
    add_executable(test_can_socket test/test_can_socket.cpp)
    target_link_libraries(test_can_socket robot_can)

    add_executable(test_canfd test/test_canfd.cpp)
    target_link_libraries(test_canfd robot_can)

    add_executable(test_direct test/test_direct.cpp)
    target_link_libraries(test_direct robot_can)

    add_executable(test_motor test/test_motor.cpp)
    target_link_libraries(test_motor robot_can)

    add_executable(test_component test/test_component.cpp)
    target_link_libraries(test_component robot_can)

    add_executable(test_multi_can test/test_multi_can.cpp)
    target_link_libraries(test_multi_can robot_can)

    add_executable(test_robot test/test_robot.cpp)
    target_link_libraries(test_robot robot_can)
endif ()