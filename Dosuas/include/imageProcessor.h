

struct Voxel {
	int x, y, z;
};


class ImageProcessor {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	std::array<int, 320 * 240> pcToImgArray(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	Voxel getNearestVoxel(std::array<pcl::PointXYZ, 240> column);
public:
	void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	std::vector<Voxel> getVoxelsForAudioSwipe(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	std::vector<std::vector<int>> getImageForChordSwipe(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
};