#include <iostream>
#include <array>
#include <vector>
#include <math.h>
#include <SFML/Audio.hpp>
#include <SFML/System.hpp>
#include "imageProcessor.h"

#define PI 3.1415f


class AudioPlayer {
	std::vector<sf::Int16> getSineWaveSamples(float frequency, float duration, int sampeRate);
	float AudioPlayer::depthToFrequency(int depth);
	void configureSoundSource(sf::SoundBuffer& buffer, sf::Sound& sound, std::vector<sf::Int16> samples,
		int sampleRate);
public:
	void playTestSound();
	void playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate = 44100);
};
