#include "audioPlayer.h"


std::vector<sf::Int16> AudioPlayer::getSineWaveSamples(float frequency, float duration, int sample_rate) {
	/* returns vector of audio samples needed to play sine wave of given
	frequency for given duration (in seconds) */
	
	const int amplitude = 20000;
	std::vector<sf::Int16> samples;
	double t = 0;
	
	for (unsigned sample = 0; sample < sample_rate * duration; sample++) {
		samples.push_back(amplitude * sin(t * 2 * PI));
		t += frequency / sample_rate;
	}
	return samples;
}


void AudioPlayer::playSoundSwipe(std::vector<int> depthValues, float duration) {
	return;
}