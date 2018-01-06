#include <iostream>
#include <array>
#include <vector>
#include <math.h>
#include <SFML/Audio.hpp>
#include <SFML/System.hpp>
#include "imageProcessor.h"

#define PI 3.1415f


class AudioPlayer {
	std::vector<sf::Int16> getSineWaveSamples(float frequency, float duration, int sample_rate=44100);
public:
	void AudioPlayer::playSoundSwipe(std::vector<int> depthValues, float duration);
};
