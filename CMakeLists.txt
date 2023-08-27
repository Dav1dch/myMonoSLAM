cmake_minimum_required(VERSION 3.23)

# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)


project(monoSLAM)

include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/"
	"./include"
	)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3")
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# project(LeastSquareProblem)
# set(G2O_ROOT /opt/homebrew/Cellar/g2o/20230223/include/g2o)

list(APPEND CMAKE_MODULE_PATH /Users/david/Codes/Apps/g2o/cmake_modules/)

#Threads
find_package(Threads QUIET)


# Pangolin
find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
Find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
find_Package(CSparse REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

include_directories(/opt/anaconda3/pkgs/suitesparse-5.10.1-h7cd81ec_1/include)
Find_Package(Ceres REQUIRED)

include_directories( ${CSPARSE_INCLUDE_DIR})
set(G2O_LIBS /usr/local/lib/libg2o_core.dylib /usr/local/lib/libg2o_stuff.dylib /usr/local/lib/libg2o_csparse_extension.dylib /usr/local/lib/libg2o_solver_csparse.dylib /usr/local/lib/libg2o_types_slam3d.dylib /usr/local/lib/libg2o_types_sim3.dylib /usr/local/lib/libg2o_types_sba.dylib)
set(G2O_LIBS_2 /Users/david/Codes/myMonoSLAM/Thirdparty/g2o/lib/libg2o.dylib)


#sophus
find_package(Sophus REQUIRED)
find_package(FMT REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen

# add_executable(gaussNewton gaussNewton.cpp)
# target_link_libraries(gaussNewton ${OpenCV_LIBS})

# add_executable(ceresCurveFitting ceresCurveFitting.cpp)
# target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

# add_executable(g2oCurveFitting g2oCurveFitting.cpp)
# target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

# add_executable(orb_cv orb_cv.cpp)
# target_link_libraries(orb_cv ${OpenCV_LIBS})


add_executable(slam src/slam.cpp src/type.cpp src/extract.cpp src/process.cpp src/tools.cpp
  src/match.cpp src/render.cpp src/optimize.cpp)
target_link_libraries(slam ${OpenCV_LIBS} ${Sophus_LIBRARIES} fmt::fmt ${Pangolin_LIBRARIES} ${CERES_LIBRARIES} ${G2O_LIBS})

add_executable(test test/main.cpp src/tools.cpp src/type.cpp)
target_link_libraries(test ${Pangolin_LIBRARIES} Threads::Threads ${OpenCV_LIBS})
