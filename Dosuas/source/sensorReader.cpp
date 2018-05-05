#include "stdafx.h"
#include "sensorReader.h"


SensorReader::SensorReader() {
	/* allocates memory and initializes device handle */

	MTF_API::mtfGetDeviceList(m_pDevInfo, &m_deviceCount);
	m_pDevHnd = m_pDevInfo[0].mtfHnd;
	// buffer initialisation
	for (int i = 0; i < IMAGE_NUM; i++) {
		m_pCameraBuf[i] = nullptr;
		m_pCameraBuf[i] = (unsigned short*)malloc(sizeof(unsigned short) * width * height);
	}
}


bool SensorReader::connect() {
	/* connects to MTF sensor; returns false if failed */

	if (MTF_API::mtfDeviceOpen(m_pDevHnd) != MTF_API::ERROR_NO) { 
		return false; 
	}
	MTF_API::mtfReadBufInit(m_pDevHnd);
	MTF_API::mtfGrabStart(m_pDevHnd);
	// adjust parameters
	MTF_API::mtfSetCheckThreshold(m_pDevHnd, 0);
	MTF_API::mtfGetDepthRange(m_pDevHnd, &minDepth, &maxDepth);
	std::cout << "depth range " << minDepth << " " << maxDepth << std::endl;
	MTF_API::mtfSetDepthRange(m_pDevHnd, minDepth, maxDepth);
	return true;
}


void SensorReader::close() {
	/* closes connection to MTF sensor gracefully */

	MTF_API::mtfGrabStop(m_pDevHnd);
	MTF_API::mtfDeviceClose(m_pDevHnd);
	for (int i = 0; i < IMAGE_NUM; i++) { // free buffer memory
		if (m_pCameraBuf[i] != NULL) {
			free(m_pCameraBuf[i]);
			m_pCameraBuf[i] = NULL;
		}
	}
}


std::array<std::array<int, 240>, 320> SensorReader::getImg() {
	/* gets depth data from sensor and returns column - row representation of image matrix 
	note: please configure undistortion filter in the CubeEye software */

	MTF_API::mtfReadFromDevice(m_pDevHnd, (unsigned short**)m_pCameraBuf, &m_stFrameInfo[m_pDevHnd]);
	std::array<std::array<int, 240>, 320> imgMat;
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			imgMat[x][y] = m_pCameraBuf[1][y * width + x] / 10;
		}
	}
	return imgMat;  // TODO: don't forget deleting after using
}