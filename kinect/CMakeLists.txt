file(GLOB KINECT_SOURCE kinect_source/*.cpp)
SET (BUILD_SHARED_LIBS  TRUE)
ADD_LIBRARY(AFEM_kinect_lib ${KINECT_SOURCE})

target_link_libraries( AFEM_kinect_lib ${OpenCV_LIBS} Kinect20)