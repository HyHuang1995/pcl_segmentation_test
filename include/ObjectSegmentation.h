# pragma  once
#include "ParamLoading.h"

#include <unistd.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

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

	void filtCloud(float leaveSize = 0.014f);
	void removePlane();
	void cluster();
private:
	const string strFilePath;
	const vector<string> vstrImageFilenamesRGB;
	const vector<string> vstrImageFilenamesDepth;
	const vector<Eigen::Isometry3d> vCamPoses;

	Eigen::Quaterniond q;
	Eigen::Isometry3d T;

	cv::Mat color;
	cv::Mat depth;

	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::ExtractIndices<PointT> extract;

	pcl::VoxelGrid<PointT> vg;
	pcl::PassThrough<PointT> pass;

	pcl::visualization::CloudViewer *viewer;
	PointCloud::Ptr pointCloud_raw;
	PointCloud::Ptr pointCloud_filtered;
	PointCloud::Ptr pointCloud_removal;
	PointCloud::Ptr pointCloud_cluster;
	PointCloud::Ptr pointCloud_plane;

	//pcl::search::KdTree<PointT>::Ptr tree;
	//std::vector<pcl::PointIndices> cluster_indices;
	//pcl::EuclideanClusterExtraction<PointT> ec;

	int colorForCluster[20][3];

	void initPlaneSeg();
};