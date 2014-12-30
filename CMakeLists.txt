cmake_minimum_required (VERSION 2.6)
project (viper2)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -Wall")
find_library(m REQUIRED)
find_library(GL REQUIRED)
find_library(GLEW REQUIRED)
find_library(noise REQUIRED)
find_library(assimp REQUIRED)

find_library(common REQUIRED)
find_library(scene REQUIRED)

find_package(SDL REQUIRED)
find_package(SDL_ttf REQUIRED)
find_package(SDL_image REQUIRED)
find_package(SDL_mixer REQUIRED)
include_directories(${SDL_INCLUDE_DIR})
include_directories(src)


add_executable(army src/army/main.cpp)
target_link_libraries(army ${M_LIB} SDL SDL_ttf SDL_image SDL_mixer GL GLEW jsoncpp noise assimp sscene common)
