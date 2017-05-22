# pragma  once 
#include "ParamLoading.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


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
private:
	const string strFilePath;
	const vector<string> vstrImageFilenamesRGB;
	const vector<string> vstrImageFilenamesDepth;

	cv::Mat color;
	cv::Mat depth;

	Eigen::Quaterniond q;
	Eigen::Isometry3d T;

	pcl::visualization::CloudViewer *viewer;
	PointCloud::Ptr pointCloud_raw;
};