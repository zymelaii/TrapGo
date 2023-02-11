cmake_minimum_required(VERSION 3.5)

project(TrapGo VERSION 0.8 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

find_package(Qt NAMES Qt6)
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

aux_source_directory(src SOURCE_FILES)
file(GLOB RC_FILES assets/*.*rc)

qt_add_executable(${PROJECT_NAME}
    MANUAL_FINALIZATION
    ${SOURCE_FILES} ${RC_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
    PRIVATE Eigen3::Eigen
)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

qt_finalize_executable(${PROJECT_NAME})
