#include <vector>
#include <array>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>


struct Voxel {
	int x, y, z;
};


class ImageProcessor {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	Voxel getVoxel(std::array<pcl::PointXYZ, 240> column);
public:
	void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
	std::vector<Voxel> getVoxelsForAudioSwipe(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
};