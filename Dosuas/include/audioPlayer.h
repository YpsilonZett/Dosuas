#include <iostream>
#include <array>
#include <vector>
#include <utility>
#include <math.h>
#include <SFML/Audio.hpp>
#include <SFML/System.hpp>
#include "imageProcessor.h"

#define PI 3.1415f


class AudioPlayer {
	std::vector<sf::Int16> getSineWaveSamples(float frequency, float duration, int sampeRate);
	std::pair<std::vector<sf::Int16>, double> getSweepSamples(float fStart, float fEnd, double duration, 
		int sampleRate, double phi0 = 0.0, int amplitude = 20000);
	std::vector<sf::Int16> getVolumeSweepSamples(float frequency, float amp1, float amp2, double duration, 
		int sampleRate);
	float depthToFrequency(int depth, bool useExpFunc = true);
	int depthToAmplitude(int depth);
	void configureSoundSource(sf::SoundBuffer& buffer, sf::Sound& sound, std::vector<sf::Int16> samples,
		int sampleRate);
	std::vector<sf::Int16> getChordSamples(std::vector<std::vector<int>> columns, float duration, int sampleRate);
	std::vector<sf::Int16> getVoxelSamples(std::vector<Voxel> voxels, float duration, int sampleRate);
public:
	void playErrorTone(float duration, int sampleRate = 44100);
	void playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate = 44100);
	void playChordSwipe(std::vector<std::vector<int>> columns, float duration, int sampleRate = 44100);
};
