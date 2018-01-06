#include "sensorReader.h"
#include "imageProcessor.h"


int main(int argc, char** argv) {
	SensorReader sr;
	ImageProcessor ip;

	sr.connect();
	for (int i = 0; i < 5; i++) {  // get a good image 
		sr.getImg();
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pImgCloud = sr.getImg();
	ip.getVoxelsForAudioSwipe(pImgCloud);
	// TODO: check for memory leaks
	return 0;
}