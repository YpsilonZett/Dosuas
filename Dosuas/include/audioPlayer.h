#include "imageProcessor.h"

#define PI 3.1415f


class AudioPlayer {
	// generate waves
	std::vector<sf::Int16> getSineWaveSamples(float frequency, float duration, int sampeRate);
	std::pair<std::vector<sf::Int16>, double> getFrequencySweepSamples(float fStart, float fEnd, double duration, 
		int sampleRate, double phi0 = 0.0, int amplitude = 20000);
	std::vector<sf::Int16> getAmplitudeSweepSamples(double freq, double amp1, double amp2, double duration, 
		int sampleRate);
	// convert A to B
	float depthToFrequency(int depth);
	float depthToAmplitude(int depth);
	double AudioPlayer::rowToFrequency(int row, int numRows);
	// prepare playing
	void configureSoundSource(sf::SoundBuffer& buffer, sf::Sound& sound, std::vector<sf::Int16> samples,
		int sampleRate);
	std::vector<std::vector<sf::Int16>> getChordSamples(std::vector<std::vector<int>> columns, float duration, 
		int sampleRate);
	std::vector<sf::Int16> getVoxelSamples(std::vector<Voxel> voxels, float duration, int sampleRate);
public:
	// play
	void playErrorTone(float duration, int sampleRate = 44100);
	void playSoundSwipe(std::vector<Voxel> voxels, float duration, int sampleRate = 44100);
	void playChordSwipe(std::vector<std::vector<int>> columns, float duration, int sampleRate = 11025);
};
