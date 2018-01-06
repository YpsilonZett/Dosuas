#include <iostream>
#include <vector>
#include <math.h>
#include <SFML/Audio.hpp>
#include <SFML/System.hpp>

#define PI 3.1415f

std::pair<float, float> getCoords(float distance, float angle)
{
	/* calculates x and y coordinate for a position described by distance and
	angle (relative to listener), 0 degrees is in front of the listener */
	double x, y;
	x = round(sin((double)angle * PI / 180.0) * distance * 100.0) / 100.0;
	y = round(cos((double)angle * PI / 180.0) * distance * 100.0) / 100.0;
	return std::make_pair((float)x, (float)y);
}


std::vector<sf::Int16> tone(float frequency, float duration, int sample_rate)
{
	/* returns vector of audio samples needed to play sine wave of given
	frequency for given duration (in seconds) */
	const int amplitude = 25000;
	std::vector<sf::Int16> samples;
	double t = 0;
	for (unsigned sample = 0; sample < sample_rate * duration; sample++) {
		samples.push_back(amplitude * sin(t * 2 * PI));
		t += frequency / sample_rate;
	}
	return samples;
}


int test_main()
{
	sf::SoundBuffer Buffer;
	std::vector<sf::Int16> samples = tone(360.0, 0.25, 44100);
	Buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);
	//Buffer.loadFromFile("bounce.wav");	
	sf::Sound sound1;
	sound1.setBuffer(Buffer);
	sound1.setRelativeToListener(true);
	sound1.setMinDistance(5.f);
	sound1.setAttenuation(10.f);

	sf::SoundBuffer Buffer2;
	std::vector<sf::Int16> samples2 = tone(540.0, 0.25, 44100);
	Buffer2.loadFromSamples(&samples2[0], samples2.size(), 1, 44100);
	sf::Sound sound2;
	sound2.setBuffer(Buffer2);
	sound2.setRelativeToListener(true);
	sound2.setMinDistance(5.f);
	sound2.setAttenuation(10.f);
	sound2.setPosition(-1.f, 3.f, 0.f);

	for (float angle = -90; angle <= 90; angle += 30) {
		std::pair<float, float> coords = getCoords(3.f, angle);
		std::cout << angle << " -> " << coords.first << " " << coords.second << std::endl;
		sound1.setPosition(coords.first, coords.second, 1.f);
		sound1.play();
		sf::sleep(sf::seconds(0.5));
		sound2.play();
		sf::sleep(sf::seconds(1));
	}
	return 0;
}
