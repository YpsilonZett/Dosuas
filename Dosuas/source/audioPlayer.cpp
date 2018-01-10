#include "audioPlayer.h"


std::vector<sf::Int16> AudioPlayer::getSineWaveSamples(float frequency, float duration, int sampleRate) {
	/* returns vector of audio samples needed to play sine wave of given
	frequency for given duration (in seconds) */
	
	std::vector<sf::Int16> samples;
	const int amplitude = 20000;
	double t = 0;

	for (unsigned sample = 0; sample < sampleRate * duration; sample++) {
		samples.push_back(amplitude * sin(t * 2 * PI));
		t += frequency / sampleRate;
	}
	return samples;
}


std::vector<sf::Int16> AudioPlayer::getTriangleWaveSamples(float frequency, float duration, int sampleRate) {
	std::vector<sf::Int16> samples;
	const int amplitude = 20000;
	const float period = sampleRate / frequency;
	float slope = 4 * amplitude * frequency;

	for (int i = 0; i < (duration * sampleRate / period); i++) {
		for (int x = 0; x < (int)(period / 2.0); x++) {
			samples.push_back(slope * x + amplitude / 2);
			//std::printf("y: %i\n", slope * x + amplitude / 2);
		}
		slope = -slope;
		for (int x = 0; x < (int)(period / 2.0); x++) {
			samples.push_back(slope * x - amplitude / 2);
			//std::printf("y: %i\n", slope * x - amplitude / 2);
		}
	}
	std::printf("Sample size: %i", samples.size());
	return samples;
}


float AudioPlayer::depthToFrequency(int depth) {
	/* converts depth in cm to frequency in Hz; 450 cm conforms 294 Hz (from 294 to 588); 
	depth is a value between 80 and 530 (sensor depth range) */

	return -0.653333 * (depth - 80) + 588;
}


void AudioPlayer::configureSoundSource(sf::SoundBuffer& buffer, sf::Sound& sound, 
	std::vector<sf::Int16> samples, int sampleRate) {
	/* configures a sound source, which is later used for playing the swipe; note that source and 
	buffer are passed by reference and initialized during the audio swipe */

	buffer.loadFromSamples(&samples[0], samples.size(), 1, sampleRate);
	sound.setBuffer(buffer);
	sound.setRelativeToListener(true);
	sound.setVolume(50);
}


std::vector<sf::Int16> AudioPlayer::getVoxelSamples(std::vector<Voxel> voxels, float duration, int sampleRate) {
	std::vector<sf::Int16> samples;
	for (int i = 0; i < voxels.size(); i++) {
		float freq = depthToFrequency(voxels[i].z);
		std::printf("Frequency: %f\n", freq);
		std::vector<sf::Int16> singleVoxelSamples = getTriangleWaveSamples(freq, duration / voxels.size(), sampleRate);
		std::printf("Samples: %i\n", singleVoxelSamples.size());
		samples.insert(samples.end(), singleVoxelSamples.begin(), singleVoxelSamples.end());
	}
	std::printf("Voxel samples size: %i", samples.size());
	return samples;
}


void AudioPlayer::playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate) {
	/* plays the audio swipe representing the image (using only one moving 3D sound source) */

	std::vector<sf::Int16> audioSamples = getVoxelSamples(voxels, duration, sampleRate);
	sf::Sound sound;
	sf::SoundBuffer buffer;
	float xPos = -16.0f;

	configureSoundSource(buffer, sound, audioSamples, sampleRate);
	//sound.setPosition(xPos, 1, 1);
	sound.play();
	sf::sleep(sf::seconds(duration));
	/*for (int i = 0; i < voxels.size(); i++) {
		xPos += voxels[i].x / 10.0f;
		sound.setPosition(xPos, 1, 1);
		std::cout << xPos << std::endl;
		sf::sleep(sf::seconds(voxels.size() / duration));
	}*/
}