#include "stdafx.h"
#include "imageProcessor.h"


Voxel ImageProcessor::getNearestVoxel(std::vector<int> column, int xCoord) {
	/* returns a voxel object from the average of the nearest k pixels */

	int i = 0, sum = 0, k = 24;
	std::sort(column.begin(), column.end(), [](int a, int b) { return a < b; });
	while ((column[i] == 0) && (i < column.size() - 1)) {
		i++;  // skip invalid pixels
	} 
	Voxel vxl;
	vxl.x = xCoord;
	vxl.y = 1;  // TODO: remove senseless y coord for beginner mode
	for (int j = 0; (j < k) && (i + j < column.size()); j++) {
		sum += column[i + j];
	}
	vxl.z = sum / k;
	return vxl;
}


Voxel ImageProcessor::getAverageVoxel(std::vector<int> column, int xCoord, int yStart) {
	/* returns weighted average voxel representing the column; weights are the distances 
	to the middle of the image, because (hypothesis) voxels in the center are more important;
	yStart is the y coordinate of the first pixel in the column */
	
	Voxel vxl;
	vxl.y = 1;
	vxl.x = xCoord;
	int sum = 0, num = 0, weight;
	for (int y = 0; y < column.size(); y++) {
		weight = IMG_HEIGHT - std::abs(IMG_HEIGHT / 2 - (yStart + y));  
		sum += weight * column[y];
		num += weight;
	}
	vxl.z = sum / num;
	return vxl;
}


std::vector<Voxel> ImageProcessor::getVoxelsForAudioSwipe(std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> imgMat) {
	/* returns voxel objects for sound generation of beginner mode */
	// TODO: remove sharp peaks (eg. with smoothing)
	std::vector<Voxel> voxels;
	for (int i = 0; i < IMG_WIDTH; i++) {
		std::vector<int> colVect(imgMat[i].begin(), imgMat[i].end());
		//Voxel vxl = getNearestVoxel(colVect, i);
		Voxel vxl = getAverageVoxel(colVect, i);
		voxels.push_back(vxl);
	}
	return voxels;
}


std::vector<std::vector<int>> ImageProcessor::getImgMatChordSwipe(std::array<std::array<int, IMG_HEIGHT>, IMG_WIDTH> rawImg,
	int numRows, int numColumns) {
	/* generates downsampled image matrix for chord mode; note: numRows and numColumns have to be divisors of
	IMG_HEIGHT and IMG_WIDTH */

	// average approach
	std::vector<std::vector<int>> columns;
	int depthSum;
	for (int x = 0; x < IMG_WIDTH; x++) {
		depthSum = 0;
		std::vector<int> column;
		for (int y = 0; y < 240; y++) {
			if ((y + 1) % (IMG_HEIGHT / numRows) == 0) {
				column.push_back((float)depthSum / (float)(IMG_HEIGHT / numRows));
				depthSum = 0;
			}
			depthSum += rawImg[x][y];
		}
		columns.push_back(column);
	}
	return columns;

	/*// nearest voxel approach
	std::vector<std::vector<int>> imgMat(numColumns, std::vector<int>(numRows));
	int rowHeight = IMG_HEIGHT / numRows, colWidth = IMG_WIDTH / numColumns;
	for (int col = 0; col < numColumns; col++) {
		for (int row = 0; row < numRows; row++) {
			std::vector<Voxel> nearestVoxels;
			for (int x = 0; x < colWidth; x++) {
				std::vector<int> subColumn;
				for (int y = 0; y < rowHeight; y++) {
					subColumn.push_back(rawImg[x + col * colWidth][y + row * rowHeight]);
				}
				nearestVoxels.push_back(getNearestVoxel(subColumn, x + col * colWidth));
			}
			Voxel nearestVoxel = *std::min_element(nearestVoxels.begin(), nearestVoxels.end(),
				[](Voxel a, Voxel b) { return a.z < b.z; });
			imgMat[col][row] = nearestVoxel.z;
		}
	}
	return imgMat;*/
}



