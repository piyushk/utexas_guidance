cmake_minimum_required(VERSION 2.6)
project(utexas_planning)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTOPENGL 1)
INCLUDE(${QT_USE_FILE})

find_package(Boost REQUIRED) # COMPONENTS filesystem program_options serialization system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(UTEXAS_PLANNING REQUIRED utexas_planning)
include_directories(${UTEXAS_PLANNING_INCLUDE_DIRS})
link_directories(${UTEXAS_PLANNING_LIBRARY_DIRS})

## foreach(library utexas_planning_common utexas_planning_core utexas_planning_planners utexas_planning_execution)
##   find_library(UTEXAS_PLANNING_LIBRARY NAMES ${library}
##     HINTS ${UTEXAS_PLANNING_LIBDIR} ${UTEXAS_PLANNING_LIBRARY_DIRS}
##   )
##   list (APPEND UTEXAS_PLANNING_LIBRARIES ${UTEXAS_PLANNING_LIBRARY})
## endforeach()

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)
link_directories(${YAML_CPP_LIBRARY_DIRS})
## find_library(YAML_CP_LIBRARY NAMES yaml-cpp
##   HINTS ${YAML_CPP_LIBDIR} ${YAML_CPP_LIBRARY_DIRS}
## )

list(APPEND CMAKE_MODULE_PATH ${utexas_planning_SOURCE_DIR}/cmake_modules)
find_package(QGLViewer REQUIRED)
include_directories(${QGLViewer_INCLUDE_DIRS})

## find_library(CLASS_LOADER_LIBRARY NAMES class_loader
##   HINTS ${CLASS_LOADER_LIBDIR} ${CLASS_LOADER_LIBRARY_DIRS}
## )
##
## find_package(Eigen3 REQUIRED)

# http://igorbarbosa.com/articles/how-to-use-opengl-freeglut-and-cmake/

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})

## include_directories(${EIGEN3_INCLUDE_DIR})
## include_directories(${CLASS_LOADER_INCLUDE_DIRS})

include_directories(include)

add_library(graph
  src/graph/graph.cpp
)
target_link_libraries(graph
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${QGLVIEWER_LIBRARY}
  ${QT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
)

add_library(mdp
  src/mdp/common.cpp
  #src/mdp/extended_structures.cpp
  src/mdp/guidance_model.cpp
  src/mdp/pdpt_solver.cpp
  #src/mdp/restricted_model.cpp
  src/mdp/single_robot_solver.cpp
  src/mdp/structures.cpp
  src/mdp/transition_model.cpp
  src/mdp/visualizer.cpp
)
target_link_libraries(mdp
  graph
  ${UTEXAS_PLANNING_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
)

add_executable(graph_viewer
  src/visualizer/graph_viewer.cpp
)
target_link_libraries(graph_viewer
  graph
  ${QGLVIEWER_LIBRARY}
  ${QT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
)

add_executable(main
  src/test/animation.cpp
  src/test/main.cpp
)
target_link_libraries(main
  ${QGLVIEWER_LIBRARY}
  ${QT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
)

install(
  TARGETS
    graph
    mdp
    main
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
