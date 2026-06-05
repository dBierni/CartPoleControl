cmake_minimum_required(VERSION 3.10)

project(CartPoleControl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# -----------------------------------
# Library
# -----------------------------------
add_library(cartpolecontrol
        src/CartPole.cpp
        src/LQR.cpp
)

target_include_directories(cartpolecontrol
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(cartpolecontrol
        PUBLIC
        Eigen3::Eigen
)

# -----------------------------------
# Executable
# -----------------------------------
add_executable(cartpole_lqr
        src/main.cpp
)

target_link_libraries(cartpole_lqr
        PRIVATE
        cartpolecontrol
)