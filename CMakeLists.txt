cmake_minimum_required(VERSION 3.0.2)
project(voroni)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_library(gdal_LIBRARIES
  gdal
)

find_package (GeographicLib REQUIRED)


#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#  src/graph_builder.cpp
#)

#add_library(graph_builder src/graph_builder.cpp)
#target_compile_options(graph_builder PRIVATE ${OpenMP_FLAGS})
#add_dependencies(graph_builder ${catkin_EXPORTED_TARGETS})
#target_link_libraries(graph_builder ${catkin_LIBRARIES} ${OpenMP_LIBS})
#install(TARGETS graph_builder
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

############
# BINARIES #
############
cs_add_executable(test_voroni
  src/test_voroni.cpp
)
target_link_libraries(test_voroni ${gdal_LIBRARIES} ${GeographicLib_LIBRARIES})



##########
# EXPORT #
##########
cs_install()
cs_export()

## Install python nodes
#set(py_nodes
#  static_mission_planner_node
#)

foreach(py_node IN LISTS py_nodes)
  catkin_install_python(PROGRAMS scripts/${py_node}.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()