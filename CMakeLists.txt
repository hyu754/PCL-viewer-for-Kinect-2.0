cmake_minimum_required(VERSION 2.8)
project( PCL_KINECT )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


include_directories(${CMAKE_SOURCE_DIR}/kinect/kinect_include/)


#third party

include_directories(${CMAKE_SOURCE_DIR}/third_party/kinect/inc/)
link_directories(${CMAKE_SOURCE_DIR}/third_party/kinect/Lib/x64/)
add_subdirectory(kinect)

add_executable( PCL_KINECT main.cpp)



#set this to your opencv version and change depending if debug or release
SET(CV_VERSION 310)
SET(CV_FILE_PRE ${CV_VERSION}d.dll)



target_link_libraries( PCL_KINECT AFEM_kinect_lib)

add_custom_target(dll_includes
	#OPENCV DLLS	
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/opencv_core${CV_FILE_PRE}            			${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/opencv_highgui${CV_FILE_PRE}            		${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/opencv_imgcodecs${CV_FILE_PRE}            		${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/opencv_imgproc${CV_FILE_PRE}            		${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/opencv_videoio${CV_FILE_PRE}           		${CMAKE_BINARY_DIR}/Debug/
	
	#PCL DLLS

	
)

add_dependencies(PCL_KINECT dll_includes)





file(GLOB VTK_LIBS
    "*.lib"
)



