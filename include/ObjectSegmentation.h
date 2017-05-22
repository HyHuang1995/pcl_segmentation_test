# pragma  once
#include "ParamLoading.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>


class ObjSeg
{
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

public:
	enum cloudType
	{RAW = 0, FILTERED = 1, REMOVAL = 2, CLUSTER = 3};
public:
	ObjSeg(ParamLoading pram);

	bool generateCloud(int index);
	void showCloud(cloudType type);

	void filtCloud(float leaveSize = 0.02f);
private:
	const string strFilePath;
	const vector<string> vstrImageFilenamesRGB;
	const vector<string> vstrImageFilenamesDepth;

	cv::Mat color;
	cv::Mat depth;

	Eigen::Quaterniond q;
	Eigen::Isometry3d T;

	pcl::VoxelGrid<PointT> vg;

	pcl::visualization::CloudViewer *viewer;
	PointCloud::Ptr pointCloud_raw;
	PointCloud::Ptr pointCloud_filtered;
};