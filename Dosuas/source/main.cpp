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
	bool ACCORD_SWEEP;
	std::cout << "Enter device mode (0/1): ";
	std::cin >> ACCORD_SWEEP;

	if (!sr.connect()) {
		std::printf("Sensor connection failed! Shutting down...");
		std::exit(1);
	};

	// configure system exit on user-interrupt 
	signal(SIGINT, gracefulShutdown);
	// process does not stop (computer does not fall asleep)
	SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED);

	while (true) {
		if (ACCORD_SWEEP == false) {
			try {
				pcl::PointCloud<pcl::PointXYZ>::Ptr pImgCloud = sr.getImg();
				std::vector<Voxel> imgVoxels = ip.getVoxelsForAudioSwipe(pImgCloud);
				ap.playSoundSwipe(imgVoxels, 5.0f);
				std::printf("Taking image and transforming to sound\n");
			}
			catch (const std::out_of_range& e) {
				std::printf("Bad image! Either the camera is too near to an object or to far from anything.\n");
				ap.playErrorTone(1.0f);
			}
		} else {
			pcl::PointCloud<pcl::PointXYZ>::Ptr pImgCloud = sr.getImg();
			std::vector<std::vector<int>> accordImage = ip.getImageForAccordSwipe(pImgCloud);
			ap.playAccordSwipe(accordImage, 5.0);
			std::printf("Taking image and transforming to sound\n");
		}
	}
	return 0;
}