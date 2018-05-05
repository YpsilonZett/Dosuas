struct Voxel {
	int x, y, z;
};


class ImageProcessor {
	Voxel getNearestVoxel(std::vector<int> column, int xCoord);
public:
	std::vector<Voxel> getVoxelsForAudioSwipe(std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> imgMat);
	std::vector<std::vector<int>> getImgMatChordSwipe(std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> rawImg,
		int numRows = 12, int numColumns = 32);
};