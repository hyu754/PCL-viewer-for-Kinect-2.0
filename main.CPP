#include <iostream>
#include "AFEM_kinect_grabber.h"
int main(void){

	AFEM_KINECT kinect_class;
	kinect_class.initialize_kinect();
	while (1){
		kinect_class.acquire_color_frame_kinect(true);
	}
	return 1;
}