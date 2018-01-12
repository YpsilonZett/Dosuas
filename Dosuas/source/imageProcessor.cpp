#include "imageProcessor.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr ImageProcessor::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) {
	/* removes error pixels (outliners), floor and ceiling from pointcloud */
	// TODO: improve algorithm (current is very naive)

	pcl::PointCloud<pcl::PointXYZ>::Ptr pVerticallyFilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCompletelyFilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;

	// remove floor and ceiling by height filtering
	pass.setInputCloud(pCloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(60.0, 240.0);
	pass.filter(*pVerticallyFilteredCloud);

	// remove measurement errors (outliners) by depth filtering
	/*pass.setInputCloud(pVerticallyFilteredCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(80.0, 530.0);
	pass.filter(*pCompletelyFilteredCloud);*/
	
	return pVerticallyFilteredCloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ImageProcessor::downSamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) {
	/* reduces the number of points in the pointcloud for (reduces resolution) */
	
	pcl::PointCloud<pcl::PointXYZ> cloudFiltered;
	float leaf = 2.75;  // calibrate for resolution change

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(pCloud);
	vg.setLeafSize(leaf, leaf, leaf); 
	vg.filter(cloudFiltered);
	return cloudFiltered.makeShared();
}


std::array<int, 320 * 240> ImageProcessor::pcToImgMat(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) {
	std::array<int, 320 * 240> imgMat;
	for (int i = 0; i < imgMat.size(); i++) {
		imgMat[i] = 0;
	}
	for (int i = 0; i < pCloud->size(); i++) {
		imgMat[pCloud->points[i].y * 320 + pCloud->points[i].x] = pCloud->points[i].z;
	}
	return imgMat;
}


Voxel ImageProcessor::getVoxel(std::array<pcl::PointXYZ, 240> column) {
	/* gets a single voxel (nearest point), wich represents the column, which is heared */
	// TODO: filtering 
	int i = 0, j = 0, sum = 0, num = 0;
	Voxel vxl;
	std::sort(column.begin(), column.end(), [](pcl::PointXYZ a, pcl::PointXYZ b) {return a.z > b.z;});
	while (column.at(i).z == 0) { i++; }  // skip invalid voxels
	for (j = i; (j < i + 10) && (j < 240); j++) {
		sum += column.at(j).z;
		num++;
	}  // throws error if all pixels are invalid
	vxl.x = column.at(j).x;
	vxl.y = 1;  // TODO: later add y to sounds
	vxl.z = sum / num;  
	return vxl;
}


void ImageProcessor::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) {
	/* displays pointcloud in simple 3D viewer */
	
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(pCloud);
	while (!viewer.wasStopped()) {}
}


std::vector<Voxel> ImageProcessor::getVoxelsForAudioSwipe(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) {
	/* calculates voxels, which are played in the audio swipe; takes image buffer (filtered pointcloud in 
	2d matrix form) as input */

	pcl::PointCloud<pcl::PointXYZ>::Ptr pFilteredCloud = filterPointCloud(pCloud);
	//showPointCloud(pFilteredCloud);
	std::array<int, 320 * 240> imgMat = pcToImgMat(pCloud);
	std::vector<Voxel> voxels;
	for (int i = 0; i < 320; i++) {
		std::array<pcl::PointXYZ, 240> col;
		for (int j = 0; j < 240; j++) {
			pcl::PointXYZ point;
			point.x = i;
			point.y = j;
			point.z = imgMat.at(i + j * 320);
			col.at(j) = point;
		}
		Voxel vxl = getVoxel(col);
		//std::printf("Voxel coordinates: %i %i %i\n", vxl.x, vxl.y, vxl.z);
		voxels.push_back(vxl);
	}
	return voxels;
}