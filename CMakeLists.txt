cmake_minimum_required(VERSION 2.8.3)
project(rviz_overlays)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rviz
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

catkin_package()

qt4_wrap_cpp(MOC_FILES
  src/minimal_overlay.h
  src/overlay_utils.h
  src/pie_plot.h
  src/pitek.h
)

set(SOURCE_FILES
  src/minimal_overlay.cpp
  src/overlay_utils.cpp
  src/pie_plot.cpp
  src/pitek.cpp
  ${MOC_FILES}
  src/plotters.h
  src/text.h
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
