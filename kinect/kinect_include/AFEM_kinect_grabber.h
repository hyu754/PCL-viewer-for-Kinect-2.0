/*



*/

#ifndef AFEM_KINECT_H
#define AFEM_KINECT_H
#include "Kinect.h"
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\core.hpp>
#if defined (WIN32) && defined (BUILD_SHARED_LIBS)
#if defined (_MSC_VER)
#pragma warning(disable: 4251)
#endif
#if defined(MyLib_EXPORT)
#define  MYLIB_EXPORT __declspec(dllexport)
#else
#define  MYLIB_EXPORT __declspec(dllimport)
#endif
#else
#define MYLIB_EXPORT
#endif

//Buffer sizes of the different sources
struct buffersize{
	unsigned int infrared;
	unsigned int color;
	unsigned int depth;
};


//Structure for sources for all of the different sensors
struct kinect_source{
	IColorFrameSource* colorsource_ptr;
	IDepthFrameSource* depthsource_ptr;
	IInfraredFrameSource* infraredsource_ptr;
};


//Structure for all the readers 
struct kinect_reader{
	IColorFrameReader* colorreader_ptr;
	IDepthFrameReader* depthreader_ptr;
	IInfraredFrameReader* infraredreader_ptr;
};


//Structure for descriptions for all of the inputs
struct kinect_description{
	IFrameDescription* colorframedescription_ptr;
	IFrameDescription* depthframedescription_ptr;
	IFrameDescription* infraredframedescription_ptr;
};

struct buffer_mat{
	cv::Mat color;
	cv::Mat depth;
	cv::Mat infrared;
};

struct image_mat{
	cv::Mat color;
	cv::Mat depth;
	cv::Mat infrared;
};


class MYLIB_EXPORT AFEM_KINECT
{
public:

	//Initializes all of the kinect variables
	int initialize_kinect();

	//Information regarding the sizes of sources, which is populated in the initialize_kinect function
	int color_w, color_h;
	int depth_w, depth_h; //This is the same as the infrared w and h
	int infrared_w, infrared_h;


	//Constructor and destructor
	AFEM_KINECT();
	~AFEM_KINECT();
private:
	//Pointer to the default Kinect sensor
	IKinectSensor* kinect_ptr;

	//Structures for source,reader and description for three input methods.
	kinect_source k_source;
	kinect_reader k_reader;
	kinect_description k_description;

	//Storing buffer sizes 
	buffersize k_buffer_size;






	//kinect coordinate mapper

	ICoordinateMapper* coordinate_mapper_ptr;

protected:

	//Opencv  matricies for buffer and image
	buffer_mat k_buffer_mat;
	image_mat k_image_mat;

	//the infrared image converted to the color image dimensions
	cv::Mat infrared_image;

	//Depthspacepoints and colorspacepoint vectors
	std::vector<DepthSpacePoint> *depthSpacePoints;
	std::vector<ColorSpacePoint> *colorSpacePoints;
	CameraSpacePoint *depth2xyz;
public:

	//Getting information from sources. Input: display image or not
	int acquire_color_frame_kinect(bool display);
	int acquire_infrared_frame_kinect(bool display);
	HRESULT acquire_depth_frame_kinect(bool display);

	//Map from color to depth space
	int map_color_to_depth(void);

	//Map from depth to camera space
	int map_depth_to_camera(void);

	//Map infrared image to image space
	int map_infrared_to_image(void);

	//Save infrared_image
	int save_infrared_image(std::string);


};



#endif //!AFEM_KINECT_H