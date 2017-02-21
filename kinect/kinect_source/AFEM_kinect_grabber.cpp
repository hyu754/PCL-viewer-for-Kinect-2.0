#include "AFEM_kinect_grabber.h"


//Initialize the kinect and its pointers
int AFEM_KINECT::initialize_kinect(){
	HRESULT hResult;
	hResult = GetDefaultKinectSensor(&kinect_ptr);


	if (FAILED(hResult)){
		std::cerr << "ERROR: GetDefaultKinectSensor " << std::endl;
		return -1;
	}


	hResult = kinect_ptr->Open();
	if (FAILED(hResult)){
		std::cerr << "ERROR: kinect failed to open " << std::endl;
		return -1;
	}


	hResult = kinect_ptr->get_ColorFrameSource(&k_source.colorsource_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}
	hResult = kinect_ptr->get_DepthFrameSource(&k_source.depthsource_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}
	hResult = kinect_ptr->get_InfraredFrameSource(&k_source.infraredsource_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}

	hResult = k_source.colorsource_ptr->OpenReader(&k_reader.colorreader_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}

	hResult = k_source.depthsource_ptr->OpenReader(&k_reader.depthreader_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}
	hResult = k_source.infraredsource_ptr->OpenReader(&k_reader.infraredreader_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}

	hResult = k_source.colorsource_ptr->get_FrameDescription(&k_description.colorframedescription_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}
	hResult = k_source.depthsource_ptr->get_FrameDescription(&k_description.depthframedescription_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}
	hResult = k_source.infraredsource_ptr->get_FrameDescription(&k_description.infraredframedescription_ptr);
	if (FAILED(hResult)){
		std::cerr << "ERROR:  " << std::endl;
		return -1;
	}

	if (FAILED(hResult)){
		std::cerr << "ERROR: initializing kinect failed " << std::endl;
		return -1;
	}

	//getting sizes of sources

	k_description.colorframedescription_ptr->get_Width(&color_w);
	k_description.colorframedescription_ptr->get_Height(&color_h);

	k_description.depthframedescription_ptr->get_Width(&depth_w);
	k_description.depthframedescription_ptr->get_Height(&depth_h);

	k_description.infraredframedescription_ptr->get_Width(&infrared_w);
	k_description.infraredframedescription_ptr->get_Height(&infrared_h);



	//getting buffer size

	k_buffer_size.color = color_w*color_h * 4 * sizeof(unsigned char);
	k_buffer_size.depth = depth_w*depth_h* sizeof(unsigned short);
	k_buffer_size.infrared = infrared_w*infrared_h*sizeof(unsigned short);

	//Initialize the buffer and image matrices

	k_buffer_mat.color = cv::Mat(color_h, color_w, CV_8UC4);
	k_buffer_mat.depth = cv::Mat(depth_h, depth_w, CV_16UC1);
	k_buffer_mat.infrared = cv::Mat(infrared_h, infrared_w, CV_16UC1);

	k_image_mat.color = cv::Mat(color_h, color_w, CV_8UC4);
	k_image_mat.depth = cv::Mat(depth_h, depth_w, CV_8UC1);
	k_image_mat.infrared = cv::Mat(infrared_h, infrared_w, CV_8UC1);
	infrared_image = cv::Mat(color_h, color_w, CV_8UC1);

	//initializing the depthspacepoints and colorspacepoints
	depthSpacePoints = new std::vector<DepthSpacePoint>(color_w*color_h);
	
	colorSpacePoints = new std::vector<ColorSpacePoint>(depth_w * depth_h);

	depth2xyz = new CameraSpacePoint[depth_w*depth_h];

	//Get mapper from kinect
	kinect_ptr->get_CoordinateMapper(&coordinate_mapper_ptr);
}

int AFEM_KINECT::acquire_color_frame_kinect(bool display){
	IColorFrame* colorframe_ptr;
	HRESULT colorResult = S_OK;
	colorResult = k_reader.colorreader_ptr->AcquireLatestFrame(&colorframe_ptr);

	if (SUCCEEDED(colorResult)){
		colorframe_ptr->CopyConvertedFrameDataToArray(k_buffer_size.color, (BYTE*)k_buffer_mat.color.data, ColorImageFormat::ColorImageFormat_Bgra);
		if (display){
			cv::imshow("color image", k_buffer_mat.color);
			cv::waitKey(1);
		}
	}

	if (colorframe_ptr != NULL)	colorframe_ptr->Release();

	return 1;
}

int AFEM_KINECT::acquire_infrared_frame_kinect(bool display){
	IInfraredFrame* infraredframe_ptr;
	HRESULT colorResult = S_OK;
	colorResult = k_reader.infraredreader_ptr->AcquireLatestFrame(&infraredframe_ptr);

	if (SUCCEEDED(colorResult)){
		infraredframe_ptr->AccessUnderlyingBuffer(&k_buffer_size.infrared, (UINT16**)&k_buffer_mat.infrared.data );
		if (display){
			
			k_buffer_mat.infrared.convertTo(k_image_mat.infrared, CV_8U, 0.00390625, 0.0);
			cv::imshow("infrared image", k_image_mat.infrared);
			cv::waitKey(1);
		}

		map_infrared_to_image();
	}

	if (infraredframe_ptr != NULL)	infraredframe_ptr->Release();

	return 1;
}

HRESULT AFEM_KINECT::acquire_depth_frame_kinect(bool display){
	IDepthFrame* depthframe_ptr;
	HRESULT colorResult = S_OK;
	colorResult = k_reader.depthreader_ptr->AcquireLatestFrame(&depthframe_ptr);

	if (SUCCEEDED(colorResult)){
		depthframe_ptr->AccessUnderlyingBuffer(&k_buffer_size.depth, (UINT16**)&k_buffer_mat.depth.data);
		if (display){

			k_buffer_mat.depth.convertTo(k_image_mat.depth, CV_8U, 0.2,0.0);
			cv::imshow("depth image", k_image_mat.depth);
			cv::waitKey(1);
		}

		//if acquiring depth frame is successful then call the following mapping functions
		map_color_to_depth();
		map_depth_to_camera();
		
	}
	else{
		return -1;
	}

	if (depthframe_ptr != NULL)	depthframe_ptr->Release();
	return S_OK;
	
}


int AFEM_KINECT::map_color_to_depth(void){
	HRESULT mapperResult = S_OK;


	//std::vector<DepthSpacePoint> depthSpacePoints222(color_w*color_h);
	mapperResult = coordinate_mapper_ptr->MapColorFrameToDepthSpace(depth_w*depth_h, (UINT16*)(k_buffer_mat.depth.data), color_w*color_h, &((*depthSpacePoints)[0]));
	if (FAILED(mapperResult)){
		std::cerr << "ERROR: color to depth mapping failed" << std::endl;
		return -1;
	}

	return 1;


}

int AFEM_KINECT::map_depth_to_camera(void){
	HRESULT mapperResult = S_OK;


	//std::vector<DepthSpacePoint> depthSpacePoints222(color_w*color_h);
	mapperResult = coordinate_mapper_ptr->MapDepthFrameToCameraSpace(depth_w*depth_h, (UINT16*)(k_buffer_mat.depth.data), depth_w*depth_h, depth2xyz);
	
	
	if (FAILED(mapperResult)){
		std::cerr << "ERROR: depth to camera mapping failed" << std::endl;
		return -1;
	}

	return 1;
}

int AFEM_KINECT::map_infrared_to_image(void){
	HRESULT mapperResult = S_OK;


	//std::vector<DepthSpacePoint> depthSpacePoints222(color_w*color_h);
	mapperResult = coordinate_mapper_ptr->MapDepthFrameToColorSpace(depth_w*depth_h, (UINT16*)(k_buffer_mat.infrared.data), depth_w*depth_h, &((*colorSpacePoints)[0]));


	if (FAILED(mapperResult)){
		std::cerr << "ERROR: infrared to image mapping failed" << std::endl;
		return -1;
	}
	infrared_image = cv::Scalar(0);
	//infrared_image = cv::Scalar(0);
	for (unsigned int i = 0; i < color_w; i = i + 1) {
		for (unsigned int j = 0; j < color_h; j = j + 1){


			int color_index = i + j*color_w;
			DepthSpacePoint colorToDepth = (*depthSpacePoints)[color_index];
			cv::Vec4b dummycolor2 = k_buffer_mat.color.at<cv::Vec4b>(j, i); //Note that the images is row by column
			int depthX = (int)(colorToDepth.X + 0.5);
			int depthY = (int)(colorToDepth.Y + 0.5);
			int depthIndex = depthX + depthY * depth_w;
			if ((depthX >= 0) && (depthX < depth_w) && (depthY >= 0) && (depthY < depth_h)) {

				infrared_image.data[color_index] =k_image_mat.infrared.data[depthIndex];
				
				//CameraSpacePoint depthToCamera = depth2xyz[depthIndex];




				
			}
		}
	}

	cv::imshow("infrared_iamge", infrared_image);
	cv::waitKey(1);

	
	

	return 1;
}


int AFEM_KINECT::save_infrared_image(std::string name_file){
	bool write_success=cv::imwrite(name_file, infrared_image);
	if (write_success == false){
		std::cerr << "ERROR:failed to write infrared image" << std::endl;
		return -1;
	}
	return 1;
}




AFEM_KINECT::AFEM_KINECT(){
	std::cout << "AFEM_KINECT class created" << std::endl;
}

AFEM_KINECT::~AFEM_KINECT(){
	std::cout << "AFEM_KINECT destroyed" << std::endl;
	if(kinect_ptr!=NULL) kinect_ptr->Release();
	if (coordinate_mapper_ptr != NULL) coordinate_mapper_ptr->Release();
	delete depth2xyz;
}