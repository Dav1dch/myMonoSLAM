cmake_minimum_required(VERSION 3.23)

project(monoSLAM)

include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/",
	"./include",
	"./src"
	)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# project(LeastSquareProblem)
# set(G2O_ROOT /opt/homebrew/Cellar/g2o/20230223/include/g2o)

list(APPEND CMAKE_MODULE_PATH /Users/david/Codes/Apps/g2o/cmake_modules/)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
Find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen

# add_executable(gaussNewton gaussNewton.cpp)
# target_link_libraries(gaussNewton ${OpenCV_LIBS})

# add_executable(ceresCurveFitting ceresCurveFitting.cpp)
# target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

# add_executable(g2oCurveFitting g2oCurveFitting.cpp)
# target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

# add_executable(orb_cv orb_cv.cpp)
# target_link_libraries(orb_cv ${OpenCV_LIBS})


add_executable(slam src/slam.cc src/extract.cc)
target_link_libraries(slam ${OpenCV_LIBS} ${CERES_LIBRARIES})
