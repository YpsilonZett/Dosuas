#include "stdafx.h"
#include "sensorReader.h"
#include "audioPlayer.h"  // also includes imageProcessor


SensorReader sr;


void gracefulShutdown(int sigNum) {
	sr.close();
	std::printf("Program stopped. Shutting down gracefully...\n");
	std::exit(0); 
}


int main(int argc, char** argv) {
	ImageProcessor ip;
	AudioPlayer ap;
	bool ADVANCED_MODE;
	float SWEEP_DURATION;
	int NUM_ROWS = 0;
	std::cout << "Mode (0 = 2D depth (beginner), 1 = 3D chords (advanced)): ";
	std::cin >> ADVANCED_MODE;
	std::cout << "Enter sweep duration (float): ";
	std::cin >> SWEEP_DURATION;
	if (ADVANCED_MODE) {
		std::cout << "Enter number of rows (different tones of chord): ";
		std::cin >> NUM_ROWS;
		if ((NUM_ROWS == 0) || (12 % NUM_ROWS != 0)) {
			std::cout << "invalid row number (has to be divisor of 12)";
			return -1;
		}
	}  // TODO: user entered number of columns 

	if (!sr.connect()) {
		std::printf("Sensor connection failed! Shutting down...");
		std::exit(1);
	};

	// configure system exit on user-interrupt 
	signal(SIGINT, gracefulShutdown);
	// process does not stop (computer does not fall asleep)
	SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED);
	while (true) {
		try {
			std::printf("Taking image and transforming to sound...\n");
			for (int i = 0; i < 3; i++) {  
				sr.getImg();  // get a good image (take multiple ones for better light exposure)
			}
			std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> imgMat = sr.getImg();
			if (ADVANCED_MODE == false) {
				std::vector<Voxel> imgVoxels = ip.getVoxelsForAudioSwipe(imgMat);
				ap.playSoundSwipe(imgVoxels, SWEEP_DURATION);
			} else {
				std::vector<std::vector<int>> chordImage = ip.getImgMatChordSwipe(imgMat, NUM_ROWS);
				ap.playChordSwipe(chordImage, SWEEP_DURATION);
			}
		} catch (const std::out_of_range& e) {  // TODO: review, if this is even possible
			std::printf("Error while processing image! Retrying...\n");
			e;
		}
	}
	return 0;
}