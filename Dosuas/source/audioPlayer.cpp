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


std::pair<std::vector<sf::Int16>, double> AudioPlayer::getSweepSamples(float fStart, float fEnd, double duration, 
	int sampleRate, double phi0, int amplitude) {
	/* generates sine sweep from fStart to fEnd; allows multiple sweeps with smooth continuous phase (use phi0
	for continuous phase, remember phi0 and set phi0 of next sweep to old phi0) */
	
	std::vector<sf::Int16> samples;
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


std::pair<std::vector<sf::Int16>, double> AudioPlayer::getVolumeSweepSamples(float frequency, float amp1,
	float amp2, double duration, int sampleRate, double phi0) {
	/* generates samples for sine sweep with constant frequency and changing amplitude */
	
	std::vector<sf::Int16> samples;
	const double numSamples = sampleRate * duration;
	double playedSamples, t, value;
	for (int i = 0; i < numSamples; i++) {
		playedSamples = ((double)i) / numSamples;  // played samples = amplitude step 
		t = duration * playedSamples;  
		value = sin(2.0 * PI * t * frequency + phi0) * (amp1 + (amp2 - amp1) * playedSamples);
		std::cout << value << ", ";
		samples.push_back(value);
	}
	double newPhi0 = 2.0 * PI * duration * amp2 + phi0;  // new starting phase 
	return std::make_pair(samples, newPhi0);
}


float AudioPlayer::depthToFrequency(int depth, bool useExpFunc) {
	/* converts depth in cm to frequency in Hz, so that larger distance means a lower sound */
	if (useExpFunc) {
		return exp((double)-(depth - 80) * 0.0045264 + 7.74066);
	} else {
		return (-20.0f / 9.0f) * depth + 1300;
	}
}


int AudioPlayer::depthToAmplitude(int depth) {
	if (depth == 0) { 
		return 0; 
	}
	return 38533.7 * exp(-0.00819751 * depth);
}


void AudioPlayer::configureSoundSource(sf::SoundBuffer& buffer, sf::Sound& sound, 
	std::vector<sf::Int16> samples, int sampleRate) {
	/* configures a sound source, which is later used for playing the swipe; note that source and 
	buffer are passed by reference and initialized during the audio swipe */

	buffer.loadFromSamples(&samples[0], samples.size(), 1, sampleRate);
	sound.setBuffer(buffer);
	sound.setRelativeToListener(true);
	sound.setVolume(50);
	sound.setAttenuation(0.0);//5);
}


std::vector<sf::Int16> AudioPlayer::getVoxelSamples(std::vector<Voxel> voxels, float duration, int sampleRate) {
	/* generates audio buffer for all voxels of the image */

	std::vector<sf::Int16> samples;
	const double singleDuration = duration / voxels.size() / 2.0;
	double phi0 = 0.0;
	float freq1, freq2;

	for (int i = 1; i < voxels.size(); i++) {
		freq1 = depthToFrequency(voxels[i-1].z);
		freq2 = depthToFrequency(voxels[i].z);
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
}


void AudioPlayer::playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate) {
	/* plays the audio swipe representing the image (using only one moving 3D sound source) */

	std::vector<sf::Int16> audioSamples = getVoxelSamples(voxels, duration, sampleRate);
	sf::Sound sound;
	sf::SoundBuffer buffer;
	float xPos = -15.0f;

	configureSoundSource(buffer, sound, audioSamples, sampleRate);
	sound.setPosition(xPos, 1, 1);
	sound.play();
	for (int i = 0; i < voxels.size(); i++) {
		xPos = -15.0f + (float)voxels[i].x / 10.0f;
		sound.setPosition(xPos, 1, 1);
		sf::sleep(sf::seconds(duration / voxels.size()));
		//std::cout, << voxels[i].z << ", ";
		//std::printf("coords: %f %f %f\n", xPos, voxels[i].y, voxels[i].z);
	}
}


void AudioPlayer::playChordSwipe(std::vector<std::vector<int>> columns, float duration, int sampleRate) {
	/* plays newer version of sound swipe, where instead of the nearest voxel, a chord of all pixel in a vertical 
	column is played; volume corresponds to distance (louder if nearer) and frequency to voxel position on y axis */
	std::array<float, 24> noteFrequencies = {
		880.00, 830.61, 783.99, 739.99, 698.46, 659.25, 622.25, 587.33, 554.37, 523.25, 493.88, 466.16, 
		440.00, 415.30, 392.00, 369.99, 349.23, 329.63, 311.13, 293.66, 277.81, 261.63, 246.94, 233.08
	};
	sf::Sound sound;
	sf::SoundBuffer buffer;
	std::vector<sf::Int16> samples;
	int depthSum = 0;
	float amp1, amp2, average1 = 0, average2 = 0;
	double phi0 = 0.0, singleDuration = duration / ((double)columns.size() / 20.0);
	std::pair<std::vector<sf::Int16>, double> sweep1, sweep2;
	/*for (int i = 1; i < columns.size(); i++) {
		depthSum += columns[i][0];
		//std::cout << "depth: " << columns[i][0] << std::endl;
		if (i % 10 != 0) {
			continue;
		}
		average2 = (float)depthSum / 10.0;
		depthSum = 0;
		amp1 = depthToAmplitude(average1);
		amp2 = depthToAmplitude(average2);
		average1 = average2;
		sweep1 = getVolumeSweepSamples(noteFrequencies[0], amp1, amp2, singleDuration, sampleRate, phi0);
		std::cout << phi0 << std::endl;
		phi0 = sweep1.second;
		samples.insert(samples.end(), sweep1.first.begin(), sweep1.first.end());	
		sweep2 = getVolumeSweepSamples(noteFrequencies[0], amp2, amp2, singleDuration, sampleRate, phi0);
		std::cout << phi0 << std::endl;
		phi0 = sweep2.second;
		samples.insert(samples.end(), sweep2.first.begin(), sweep2.first.end());
	}*/

	std::pair<std::vector<sf::Int16>, double> sweep, s, s2;
	sweep = getVolumeSweepSamples(440, 1000, 20000, duration, sampleRate);
	s = getVolumeSweepSamples(440, 20000, 20000, duration, sampleRate, sweep.second);
	s2 = getVolumeSweepSamples(440, 20000, 7000, duration, sampleRate, s.second);
	samples = sweep.first;
	samples.insert(samples.end(), s.first.begin(), s.first.end());
	samples.insert(samples.end(), s2.first.begin(), s2.first.end());

	configureSoundSource(buffer, sound, samples, sampleRate);
	sound.play();
	sf::sleep(sf::seconds(duration));
	sf::sleep(sf::seconds(0.5));
}