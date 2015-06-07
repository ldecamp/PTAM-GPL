cmake_minimum_required(VERSION 2.8.3)
project(ptam)

# add_custom_target(
#   DEPS ALL
#   COMMAND echo Building all dependencies
#   COMMAND cd ${PROJECT_SOURCE_DIR}/thirdparty && make
# )

find_package(OpenCV REQUIRED)
find_package(agast REQUIRED)
find_package(briskcvd REQUIRED)

#Define video binding
set(PTAM_VIDEO_SOURCE_FILES 
	src/videoSources/VideoSource_Linux_OpenCV.cc)

set(PTAM_HEADER_FILES
	include/ptam/ARDriver.h  
	include/ptam/ATANCamera.h  
	include/ptam/Bundle.h  
	include/ptam/CalibCornerPatch.h  
	include/ptam/CalibImage.h  
	include/ptam/CameraCalibrator.h  
	include/ptam/EyeGame.h  
	include/ptam/GLWindow2.h  
	include/ptam/GLWindowMenu.h  
	include/ptam/HomographyInit.h  
	include/ptam/KeyFrame.h  
	include/ptam/LevelHelpers.h  
	include/ptam/MEstimator.h  
	include/ptam/Map.h  
	include/ptam/MapMaker.h  
	include/ptam/MapPoint.h  
	include/ptam/MapViewer.h  
	include/ptam/MatchFinder.h  
	include/ptam/MiniPatch.h  
	include/ptam/OpenGL.h
	include/ptam/PatchFinder.h
	include/ptam/Relocaliser.h
	include/ptam/ShiTomasi.h
	include/ptam/SmallBlurryImage.h
	include/ptam/SmallMatrixOpts.h
	include/ptam/Statistics.h
	include/ptam/System.h
	include/ptam/Tracker.h
	include/ptam/TrackerData.h
	include/ptam/VideoSource.h
)

set(PTAM_SOURCE_FILES         
	src/main.cc
	src/GLWindow2.cc             
	src/GLWindowMenu.cc
	src/System.cc
	src/ATANCamera.cc
	src/KeyFrame.cc        
  	src/MapPoint.cc
  	src/Map.cc
	src/MapViewer.cc 
	src/SmallBlurryImage.cc              
	src/ShiTomasi.cc       
	src/HomographyInit.cc
	src/MapMaker.cc
	src/Bundle.cc
	src/PatchFinder.cc
	src/Relocaliser.cc
	src/MiniPatch.cc    
	src/MatchFinder.cc    
	src/Tracker.cc
	src/ARDriver.cc
	src/EyeGame.cc
	src/Statistics.cc)
	
SET(CALIBRATOR_FILES     
  src/CameraCalibrator.cc 
  src/CalibCornerPatch.cc           
  src/CalibImage.cc            
  src/GLWindowMenu.cc
  src/GLWindow2.cc	
  src/ATANCamera.cc)	

include_directories(
  ${PROJECT_SOURCE_DIR}/include 
  ${agast_INCLUDE_DIRS}
  ${briskcvd_INCLUDE_DIRS})  

set(PTAM_LIBRARIES GL glut GLU cvd GVars3 lapack)

add_definitions(-DCVD_HAVE_TOON)
add_definitions(-DKF_REPROJ)
set(CMAKE_BUILD_TYPE Release)

#set output path to bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# PTAM executable
add_executable(ptam ${PTAM_SOURCE_FILES} ${PTAM_VIDEO_SOURCE_FILES} ${PTAM_HEADER_FILES})

set_property(TARGET ptam APPEND_STRING PROPERTY COMPILE_FLAGS "-D_LINUX -D_REENTRANT -g -Wall -march=native ")
IF(CMAKE_BUILD_TYPE MATCHES Release)
 set_property(TARGET ptam APPEND_STRING PROPERTY COMPILE_FLAGS "-O3 ")
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsee -funsafe-loop-optimizations -fno-signed-zeros -fno-math-errno -funroll-loops")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# add_dependencies(ptam DEPS ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(ptam ${PTAM_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(ptam ${PTAM_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ptam ${PTAM_LIBRARIES} ${agast_LIBRARIES})
target_link_libraries(ptam ${PTAM_LIBRARIES} ${briskcvd_LIBRARIES})

# Camera calibrator 
add_executable(cameracalibrator ${CALIBRATOR_FILES} ${PTAM_VIDEO_SOURCE_FILES} ${PTAM_HEADER_FILES})
# add_dependencies(cameracalibrator DEPS ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(cameracalibrator ${PTAM_LIBRARIES} ${OpenCV_LIBS})