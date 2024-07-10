cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(DeckTX LANGUAGES CXX C VERSION 1.0.0)

# Dependencies
set(BUILD_TESTS OFF)
set(BUILD_SHARED_LIBS OFF)

# CPM
include(external/CPM.cmake)

CPMAddPackage("gh:fmtlib/fmt#11.0.1")
CPMAddPackage("gh:libsdl-org/SDL#9406a9d527fc4436ade12ca3ce1e6c2e617e8ead")

# IMGUI
include(external/imgui.cmake)

# DeckTX
add_executable(DeckTX src/main.cpp src/crsf/crsf.cpp src/serial/serial.cpp)
target_include_directories(DeckTX PRIVATE src)

set_property(TARGET DeckTX PROPERTY CXX_STANDARD 23)
target_link_libraries(DeckTX fmt imgui SDL3::SDL3)

# Install
if(WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
endif()

install(TARGETS DeckTX DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" "${PROJECT_SOURCE_DIR}/README.md" DESTINATION "${CMAKE_INSTALL_PREFIX}")