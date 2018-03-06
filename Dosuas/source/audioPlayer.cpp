#include "stdafx.h"
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


std::pair<std::vector<sf::Int16>, double> AudioPlayer::getFrequencySweepSamples(float fStart, float fEnd, double duration, 
	int sampleRate, double phi0, int amplitude) {
	/* generates sine sweep from fStart to fEnd; allows multiple sweeps with smooth continuous phase (use phi0
	for continuous phase, remember phi0 and set phi0 of next sweep to old phi0) */
	//TODO: better sweep

	std::vector<sf::Int16> samples;
	const int nValues = sampleRate * duration;
	const double actualSamplingTime = (double)nValues / (double)sampleRate;
	for (int i = 0; i < nValues; i++) {
		double delta = ((double)i) / ((double)nValues);
		double t = actualSamplingTime * delta;
		double phase = 2.0 * PI * t * ((double)(fStart + (fEnd - fStart) * delta) / 2.0);
		double value = sin(phase + phi0);
		samples.push_back(amplitude * value);
	}
	double phase = 2.0 * PI * actualSamplingTime * ((double)(fStart + (fEnd - fStart)) / 2.0);
	return std::make_pair(samples, phi0 + phase);
}


std::vector<sf::Int16> AudioPlayer::getAmplitudeSweepSamples(double freq, double amp1, double amp2,
	double duration, int sampleRate) {
	/* generates samples for sine sweep with constant frequency and changing amplitude */

	std::vector<sf::Int16> samples;
	int numCycles = freq * duration; 
	int numSamples = ((double)numCycles / freq) * sampleRate;
	double t, a, y;
	for (int i = 0; i < numSamples; i++) {
		t = (double)i / (double)numSamples;  // time parameter
		a = amp1 + t * (amp2 - amp1);  // linear amplitude function of time 
		y = a * sin(2.0 * PI * t * numCycles);  // y = f(x) wave function
		samples.push_back(y);
	}
	return samples;
}


float AudioPlayer::depthToFrequency(int depth) {
	/* represents depth in cm as frequency in Hz, so that larger distance means a lower sound */
	if ((depth < 80) || (depth > 530)) {
		return 300;
	}
	return 3303.63 * exp(-0.0045264 * depth);
}


float AudioPlayer::depthToAmplitude(int depth) {
	/* represents depth in cm as volume (unknown amplitude unit), so that larger distances mean are more silent */
	if ((depth < 80) || (depth > 530)) {
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
		sweepToThisVoxel = getFrequencySweepSamples(freq1, freq2, singleDuration, sampleRate, phi0);
		samples.insert(samples.end(), sweepToThisVoxel.first.begin(), sweepToThisVoxel.first.end());
		phi0 = sweepToThisVoxel.second;

		// current frequency remains constant over some time
		thisVoxel = getFrequencySweepSamples(freq2, freq2, singleDuration, sampleRate, phi0);
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
	/* plays the audio swipe representing the image (using only one moving 3D sound source); beginner mode */

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
	}
}


std::vector<std::vector<sf::Int16>> AudioPlayer::getChordSamples(std::vector<std::vector<int>> columns, float duration, 
	int sampleRate) {
	/* generates samples needed to play the chord swipe (advanced mode) by first generating sine waves with 
	changing amplitudes of different frequencies and then adding them up to get the sequence of chords */

	std::array<double, 24> noteFrequencies = {
		880.00, 830.61, 783.99, 739.99, 698.46, 659.25, 622.25, 587.33, 554.37, 523.25, 493.88, 466.16,
		440.00, 415.30, 392.00, 369.99, 349.23, 329.63, 311.13, 293.66, 277.81, 261.63, 246.94, 233.08
	};
	std::vector<sf::Int16> connectingSweep, constantTone;
	std::vector<std::vector<sf::Int16>> resultChordSamples;
	int depthSum = 0;
	float average1 = 0, average2 = 0;
	double amp1, amp2, singleDuration = (double)duration / (double)((double)columns.size() / 5.0);
	for (int j = 0; j < columns.at(0).size(); j++) {
		std::vector<sf::Int16> samples;
		for (int i = 1; i < columns.size(); i++) {
			depthSum += columns.at(i).at(j);
			if (i % 10 != 0) {
				continue;
			}
			average2 = (float)depthSum / 10.0;
			depthSum = 0;
			amp1 = depthToAmplitude(average1);
			amp2 = depthToAmplitude(average2);
			average1 = average2;
			connectingSweep = getAmplitudeSweepSamples(noteFrequencies.at(j), amp1, amp2, singleDuration, sampleRate);
			samples.insert(samples.end(), connectingSweep.begin(), connectingSweep.end());
			constantTone = getAmplitudeSweepSamples(noteFrequencies.at(j), amp2, amp2, singleDuration, sampleRate);
			samples.insert(samples.end(), constantTone.begin(), constantTone.end());
		}
		/*for (int i = 0; i < samples.size(); i++) {
			resultChordSamples.at(i) += samples.at(i);
		}*/
		resultChordSamples.push_back(samples);
	}
	return resultChordSamples;
}


void AudioPlayer::playChordSwipe(std::vector<std::vector<int>> columns, float duration, int sampleRate) {
	/* plays newer swipe version (advanced mode) using chords, volume and 3d position */

	/*sf::Sound sound;
	sf::SoundBuffer buffer;
	std::vector<sf::Int16> samples = getChordSamples(columns, duration, sampleRate);
	configureSoundSource(buffer, sound, samples, sampleRate);
	sound.play();
	sf::sleep(sf::seconds(duration));*/
	std::vector<sf::Sound> sounds(24);
	std::vector<sf::SoundBuffer> buffers(24);
	std::vector<std::vector<sf::Int16>> samples = getChordSamples(columns, duration, sampleRate);
	
	for (int i = 0; i < samples.size(); i++) {
		configureSoundSource(buffers[i], sounds[i], samples[i], sampleRate);
	}
	for (int i = 0; i < sounds.size(); i++) {
		sounds[i].play();
	}
	clock_t begin = clock();
	for (float xPos = -12.0f; xPos < 12; xPos++) {
		for (int i = 0; i < sounds.size(); i++) {
			sounds[i].setPosition(xPos, 1, 1);
		}
		//std::cout << "pos: " << xPos << std::endl;
		sf::sleep(sf::seconds(duration / 24.0));
	}
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "time: " << elapsed_secs << std::endl;
}