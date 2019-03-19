add_library(muti_planning SHARED roadmap.cpp)
target_link_libraries(muti_planning ${catkin_LIBRARIES})

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(agent agent.cpp)
target_link_libraries(agent muti_planning)
#add_dependencies(agent muti_planning_gencpp)
add_dependencies(
agent
${PROJECT_NAME}_generate_messages_cpp
${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planner planner.cpp)
target_link_libraries(planner muti_planning)
add_dependencies(
planner
${PROJECT_NAME}_generate_messages_cpp
${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

##add_dependencies(planner muti_planning_gencpp)
