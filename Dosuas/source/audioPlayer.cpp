#include "audioPlayer.h"


std::vector<sf::Int16> AudioPlayer::getSineWaveSamples(float frequency, float duration, int sampleRate) {
	/* returns vector of audio samples needed to play sine wave of given
	frequency for given duration (in seconds) */
	
	const int amplitude = 20000;
	std::vector<sf::Int16> samples;
	double t = 0;
	
	for (unsigned sample = 0; sample < sampleRate * duration; sample++) {
		samples.push_back(amplitude * sin(t * 2 * PI));
		t += frequency / sampleRate;
	}
	return samples;
}


std::vector<sf::Int16> AudioPlayer::getSweepSamples(double f_start, double f_end, double interval, int n_steps) {
	std::vector<sf::Int16> samples;
	for (int i = 0; i < n_steps; ++i) {
		double delta = i / (float)n_steps;
		double t = interval * delta;
		double phase = 2 * PI * t * (f_start + (f_end - f_start) * delta / 2);
		while (phase > 2 * PI) { phase -= 2 * PI; }
		samples.push_back(3 * sin(phase));
		//printf("%f %f %f", t, phase * 180 / PI, 3 * sin(phase));
	}
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
	//std::vector<sf::Int16> samples = getSweepSamples(1, 10, 5, 1000);
	/*const float singleDuration = duration / voxels.size();
	const int singleSampleRate = singleDuration * sampleRate;

	for (int i = 0; i < voxels.size(); i++) {
		float freq = depthToFrequency(voxels[i].z);
		std::vector<sf::Int16> partialSamples = getSineWaveSamples(freq, singleDuration, singleSampleRate);
		samples.insert(samples.end(), partialSamples.begin(), partialSamples.end());
	}*/
	std::vector<sf::Int16> samples = getSineWaveSamples(440, 1.0f, sampleRate);
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