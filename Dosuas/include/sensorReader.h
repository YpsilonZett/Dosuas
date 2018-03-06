#define IMAGE_NUM 2


class SensorReader {
	unsigned short *m_pCameraBuf[IMAGE_NUM]; // temporary image buffer
	int m_deviceCount = 0;
	MTF_API::mtfHandle m_pDevHnd; // control device handle
	MTF_API::mtfDeviceInfo m_pDevInfo[MAX_DEVICE]; // device information structure
	MTF_API::mtfFrameInfo m_stFrameInfo[MAX_DEVICE]; // frame information structure

public:
	int width = 320;
	int height = 240;
	int minDepth, maxDepth;

	SensorReader();
	bool connect();
	void close();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getImg();
};
