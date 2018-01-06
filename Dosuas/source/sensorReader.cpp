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


pcl::PointCloud<pcl::PointXYZ>::Ptr SensorReader::getImg() {
	/* gets depth data from sensor and returns pointcloud for 3d space representation */

	MTF_API::mtfReadFromDevice(m_pDevHnd, (unsigned short**)m_pCameraBuf, &m_stFrameInfo[m_pDevHnd]);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = width;
	cloud->height = height;
	cloud->points.resize(cloud->width * cloud->height);

	int i = 0;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			cloud->points[i].x = x;
			cloud->points[i].y = y;
			cloud->points[i].z = m_pCameraBuf[1][y * width + x] / 10;  // TODO: configure according to max distance and voxelgrid
			i++;
		}
	}
	return cloud;  // TODO: don't forget deleting after using
}