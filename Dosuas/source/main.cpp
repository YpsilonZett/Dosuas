#include "sensorReader.h"
#include "audioPlayer.h"  // also includes imageProcessor


int main(int argc, char** argv) {
	SensorReader sr;
	ImageProcessor ip;
	AudioPlayer ap;

	sr.connect();
	for (int i = 0; i < 5; i++) {  // get a good image 
		sr.getImg();
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pImgCloud = sr.getImg();
	std::vector<Voxel> imgVoxels = ip.getVoxelsForAudioSwipe(pImgCloud);
	ap.playSoundSwipe(imgVoxels, 10);
	return 0;
}