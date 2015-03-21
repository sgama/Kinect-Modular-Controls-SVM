cmake_minimum_required(VERSION 2.8)
project(exercise1)

#find_package( OpenCV REQUIRED )
find_package( OpenGL REQUIRED)
find_package( GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS}) #${OpenCV_INCLUDE_DIRS}

set(BIN_SOURCES
${PROJECT_SOURCE_DIR}/main.cpp
${PROJECT_SOURCE_DIR}/Gesture.hpp
${PROJECT_SOURCE_DIR}/OneDollarRecognizer.hpp
)

add_executable(${PROJECT_NAME}_bin ${BIN_SOURCES})
target_link_libraries(${PROJECT_NAME}_bin ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ) #${OpenCV_LIBS}
