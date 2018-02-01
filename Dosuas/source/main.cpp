#include <signal.h>
#include "sensorReader.h"
#include "audioPlayer.h"  // also includes imageProcessor


SensorReader sr;


void gracefulShutdown(int sigNum) {
	sr.close();
	std::printf("Program stopped. Shutting down gracefully...\n");
	std::exit(0);  // TODO: improve bad-style exit
}


int main(int argc, char** argv) {
	ImageProcessor ip;
	AudioPlayer ap;

	if (!sr.connect()) {
		std::printf("Sensor connection failed! Shutting down...");
		throw std::runtime_error("Sensor connection failed!");
	};

	// configure system exit on user-interrupt 
	signal(SIGINT, gracefulShutdown);
	// process does not stop (computer does not fall asleep)
	SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED);

	while (true) {
		for (int i = 0; i < 3; i++) {  // take multiple images for better integration (light exposure)
			sr.getImg();
		}
		try {
			pcl::PointCloud<pcl::PointXYZ>::Ptr pImgCloud = sr.getImg();
			std::vector<Voxel> imgVoxels = ip.getVoxelsForAudioSwipe(pImgCloud);
			ap.playSoundSwipe(imgVoxels, 5.0f);
			std::printf("Taking image and transforming to sound\n");
		} catch (const std::out_of_range& e) {
			std::printf("Bad image! Either the camera is too near to an object or to far from anything.\n");
			ap.playErrorTone(1.0f);
		}
	}
	return 0;
}