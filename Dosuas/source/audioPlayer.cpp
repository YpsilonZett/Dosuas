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


void AudioPlayer::playTestSound() {
	sf::Sound sound;
	sf::SoundBuffer buffer;
	std::vector<sf::Int16> samples;
	configureSoundSource(buffer, sound, samples, 44100);
}


void AudioPlayer::playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate) {
	/* plays the audio swipe representing the image (using only one moving 3D sound source) */

	std::vector<sf::Int16> audioSamples;
	float dur = duration / voxels.size();

	for (int i = 0; i < voxels.size(); i++) {
		float freq = depthToFrequency(voxels[i].z);	
		std::vector<sf::Int16> voxelSamples = getSineWaveSamples(freq, dur, sampleRate);
		audioSamples.insert(audioSamples.end(), voxelSamples.begin(), voxelSamples.end());
	}
	//audioSamples = getSineWaveSamples(440, duration, sampleRate);

	sf::Sound sound;
	sf::SoundBuffer buffer;
	float xPos = voxels.size() / -20.0f;

	configureSoundSource(buffer, sound, audioSamples, sampleRate);
	sound.setPosition(xPos, 1, 1);
	sound.play();
	std::cout << "Size: " << voxels.size() << " " << voxels.size() / -20.0f << std::endl;
	for (int i = 0; i < voxels.size(); i++) {
		xPos = voxels[i].x / 10.0f + voxels.size() / -20.0f;
		sound.setPosition(xPos, 1, 1);
		std::cout << xPos << std::endl;
		sf::sleep(sf::seconds(dur));
	}
}