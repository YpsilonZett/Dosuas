#include "audioPlayer.h"


std::vector<sf::Int16> AudioPlayer::getSineWaveSamples(float frequency, float duration, int sampleRate) {
	/* returns vector of audio samples needed to play sine wave of given
	frequency for given duration (in seconds) */

	std::vector<sf::Int16> samples;
	const int amplitude = 20000;

	for (int i = 0; i < duration * sampleRate; i++) {
		samples.push_back(amplitude * sin(2 * PI * i * frequency / sampleRate));
	}
	return samples;
}


std::vector<sf::Int16> AudioPlayer::getTriangleWaveSamples(float frequency, float duration, int sampleRate) {
	std::vector<sf::Int16> samples;
	const int amplitude = 5000;
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
	//std::printf("Sample size: %i", samples.size());
	return samples;
}


std::pair<std::vector<sf::Int16>, double> AudioPlayer::getSweepSamples(float fStart, float fEnd, double duration, 
	int sampleRate, double phi0) {
	/* generates sine sweep from fStart to fEnd; allows multiple sweeps with smooth continuous phase */
	
	std::vector<sf::Int16> samples;
	const int amplitude = 20000;
	const int nValues = sampleRate * duration;
	const double actualSamplingTime = (double)nValues / (double)sampleRate;
	//std::printf("sampling time: %f\tnValues: %i\n", actualSamplingTime, nValues);
	for (int i = 0; i < nValues; i++) {
		double delta = ((double)i) / ((double)nValues);
		double t = actualSamplingTime * delta;
		double phase = 2.0 * PI * t * ((double)(fStart + (fEnd - fStart) * delta) / 2.0);
		double value = sin(phase + phi0);
		//std::printf("time : %f\tvalue : %f\tdelta : %f\tphase : %f\n", t, value, delta, phase);
		samples.push_back(amplitude * value);
	}
	double phase = 2.0 * PI * actualSamplingTime * ((double)(fStart + (fEnd - fStart)) / 2.0);
	return std::make_pair(samples, phi0 + phase);
}


float AudioPlayer::depthToFrequency(int depth) {
	
}

void AudioPlayer::configureSoundSource(sf::SoundBuffer& buffer, sf::Sound& sound, 
	std::vector<sf::Int16> samples, int sampleRate) {
	/* configures a sound source, which is later used for playing the swipe; note that source and 
	buffer are passed by reference and initialized during the audio swipe */

	buffer.loadFromSamples(&samples[0], samples.size(), 1, sampleRate);
	sound.setBuffer(buffer);
	sound.setRelativeToListener(true);
	sound.setVolume(50);
	sound.setAttenuation(0.5);
}


std::vector<sf::Int16> AudioPlayer::getVoxelSamples(std::vector<Voxel> voxels, float duration, int sampleRate) {
	/* generates audio buffer for all voxels of the image */

	std::vector<sf::Int16> samples;
	const double singleDuration = duration / voxels.size() / 2.0;
	double phi0 = 0.0;

	for (int i = 1; i < voxels.size(); i++) {
		float freq1 = depthToFrequency(voxels[i-1].z);
		float freq2 = depthToFrequency(voxels[i].z);
		std::pair<std::vector<sf::Int16>, double> sweepToThisVoxel, thisVoxel;

		// predecessor sweeps to current frequency
		sweepToThisVoxel = getSweepSamples(freq1, freq2, singleDuration, sampleRate, phi0);
		samples.insert(samples.end(), sweepToThisVoxel.first.begin(), sweepToThisVoxel.first.end());
		phi0 = sweepToThisVoxel.second;

		// current frequency remains constant over some time
		thisVoxel = getSweepSamples(freq2, freq2, singleDuration, sampleRate, phi0);
		samples.insert(samples.end(), thisVoxel.first.begin(), thisVoxel.first.end());
		phi0 = thisVoxel.second;
	}
	/*std::pair<std::vector<sf::Int16>, double> samples1 = getSweepSamples(500.0f, 1000.0f, 5.0f, 44100);
	samples = samples1.first;
	std::pair<std::vector<sf::Int16>, double> samples2 = getSweepSamples(1000.0f, 500.0f, 5.0f, 44100, samples1.second);
	samples.insert(samples.end(), samples2.first.begin(), samples2.first.end());*/
	return samples;
}


void AudioPlayer::playErrorTone(float duration, int sampleRate) {
	/* plays a special tone for indicating too few or too far distance from an object */

	sf::Sound sound;
	sf::SoundBuffer buffer;
	std::vector<sf::Int16> samples = getSineWaveSamples(1000, duration, sampleRate);
	configureSoundSource(buffer, sound, samples, sampleRate);
	sound.play();
	sf::sleep(sf::seconds(duration));
	sf::sleep(sf::seconds(0.5));
}


void AudioPlayer::playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate) {
	/* plays the audio swipe representing the image (using only one moving 3D sound source) */

	std::vector<sf::Int16> audioSamples = getVoxelSamples(voxels, duration, sampleRate);
	sf::Sound sound;
	sf::SoundBuffer buffer;
	float xPos = -16.0f;

	configureSoundSource(buffer, sound, audioSamples, sampleRate);
	sound.setPosition(xPos, 1, 1);
	sound.play();
	for (int i = 0; i < voxels.size(); i++) {
		xPos = -16.0f + voxels[i].x / 10.0f;
		sound.setPosition(xPos, 1, 1);
		sf::sleep(sf::seconds(duration / voxels.size()));
		//std::printf("coords: %f %f %f", xPos, voxels[i].y, voxels[i].z);
	}
	sf::sleep(sf::seconds(0.5));
}