#include "sensorReader.h"
#include "audioPlayer.h"  // also includes imageProcessor


int main(int argc, char** argv) {
	SensorReader sr;
	ImageProcessor ip;
	AudioPlayer ap;
	sr.connect();
	while (true) {
		for (int i = 0; i < 5; i++) {  // get a good image 
			sr.getImg();
		}
		try {
			pcl::PointCloud<pcl::PointXYZ>::Ptr pImgCloud = sr.getImg();
			std::vector<Voxel> imgVoxels = ip.getVoxelsForAudioSwipe(pImgCloud);
			ap.playSoundSwipe(imgVoxels, 5.0f);
			std::printf("\nStarting again\n\n");
		} catch (const std::out_of_range& e) {
			std::printf("Bad image! Either the camera is too near to an object or to far from anything.\n");
			ap.playErrorTone(1.0f);
		}
	}
	return 0;
}