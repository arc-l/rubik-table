cmake_minimum_required(VERSION 3.16)
project(rubik CXX)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
#find_package(yaml-cpp REQUIRED)
#add_subdirectory(./graph)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )




# set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")

# add_executable(test common.cpp testMain.cpp rubik13.cpp labp.cpp mp.cpp umapf.cpp mapf_post.cpp rubik.cpp rubik12.cpp formation.cpp search.cpp lap.cpp)

#target_include_directories(test "./")
# target_compile_features(test PUBLIC cxx_std_17)
# target_link_libraries(test yaml-cpp ${Boost_LIBRARIES})

add_executable(main common.cpp main.cpp rubik13.cpp labp.cpp mp.cpp umapf.cpp rubik12.cpp rubik_obs.cpp)
target_compile_features(main PUBLIC cxx_std_17)
# set(GUROBI_INC /opt/gurobi912/linux64/include/)
# set(GUROBI_LIB "/opt/gurobi912/linux64/lib/libgurobi_c++.a" "/opt/gurobi912/linux64/lib/libgurobi91.so")
#target_include_directories(main  PRIVATE ${GUROBI_INC})
target_link_libraries(main yaml-cpp ${Boost_LIBRARIES})# ${GUROBI_LIB})



# add_executable(mapf_post common.cpp  postMain.cpp mapf_post.cpp labp.cpp)
# target_compile_features(mapf_post PUBLIC cxx_std_17)
# target_link_libraries(mapf_post yaml-cpp ${Boost_LIBRARIES})
