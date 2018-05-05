#include "stdafx.h"
#include "imageProcessor.h"


Voxel ImageProcessor::getNearestVoxel(std::vector<int> column, int xCoord) {
	/* returns a voxel object from the average of the nearest k pixels */

	int i = 0, j = 0, sum = 0, num = 0;
	Voxel vxl;
	std::sort(column.begin(), column.end());
	while (i < column.size()) {  // skip 0s (invalid pixels)
		if (column[i] != 0) {
			break;
		}
		i++;
	}
	for (j = i; (j < i + 10) && (j < column.size()); j++) {  // get average over the 10 nearest pixels
		sum += column[j];
		num++;
	}
	vxl.x = xCoord;
	vxl.y = 1;
	if (num == 0) {
		vxl.z = 0;  // all voxels of this column are invalid
	}
	else {
		vxl.z = sum / num;
	}
	/*int i = 0, sum = 0, k = 10;
	std::sort(column.begin(), column.end());
	while ((column[i] == 0) && (i < column.size() - 1)) {
		i++;  // skip invalid pixels
	} 
	Voxel vxl;
	vxl.x = xCoord;
	vxl.y = 1;  // TODO: remove senseless y coord for beginner mode
	for (int j = 0; (j < k) && (i + j < column.size()); j++) {
		sum += column[i + j];
	}
	vxl.z = sum / k;*/
	return vxl;
}


std::vector<Voxel> ImageProcessor::getVoxelsForAudioSwipe(std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> imgMat) {
	/* returns voxel objects for sound generation of beginner mode */

	std::vector<Voxel> voxels;
	for (int i = 0; i < IMG_WIDTH; i++) {
		std::vector<int> colVect(imgMat[i].begin(), imgMat[i].end());
		Voxel vxl = getNearestVoxel(colVect, i);
		voxels.push_back(vxl);
	}
	return voxels;
}


std::vector<std::vector<int>> ImageProcessor::getImgMatChordSwipe(std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> rawImg, 
	int numRows, int numColumns) {
	/* generates downsampled image matrix for chord mode; note: numRows and numColumns have to be divisors of
	IMG_HEIGHT and IMG_WIDTH */

	std::vector<std::vector<int>> imgMat(numColumns, std::vector<int>(numRows));
	int rowHeight = IMG_HEIGHT / numRows, colWidth = IMG_WIDTH / numColumns;
	for (int col = 0; col < numColumns; col++) {
		for (int row = 0; row < numRows; row++) {
			std::vector<Voxel> nearestVoxels;
			for (int x = 0; x < colWidth; x++) {
				std::vector<int> subColumn;
				for (int y = 0; y < rowHeight; y++) {
					subColumn.push_back(rawImg[x][y + row * rowHeight]);
				}
				nearestVoxels.push_back(getNearestVoxel(subColumn, x + col * colWidth));
			}
			Voxel nearestVoxel = *std::min_element(nearestVoxels.begin(), nearestVoxels.end(), 
				[](Voxel a, Voxel b) { return a.z < b.z; });
			imgMat[col][row] = nearestVoxel.z;
		}
	}
	return imgMat;
}