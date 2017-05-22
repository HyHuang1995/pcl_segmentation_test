#include "ObjectSegmentation.h"

#include <iostream>

ObjSeg::ObjSeg(ParamLoading param):
	strFilePath(param.strFilePath),
	vstrImageFilenamesRGB(param.vstrImageFilenamesRGB),
	vstrImageFilenamesDepth(param.vstrImageFilenamesDepth),
	q(Eigen::Quaterniond( 1, 0, 0, 0 )), T(Eigen::Isometry3d(q))
{
	T.pretranslate( Eigen::Vector3d( 0, 0, 0 ));
	viewer = new pcl::visualization::CloudViewer("viewer");
	pointCloud_raw = PointCloud::Ptr(new PointCloud) ;
	pointCloud_filtered = PointCloud::Ptr(new PointCloud) ;

	cout << endl;
	cout << "Object segmentation module constructed!" << endl;
}

bool ObjSeg::generateCloud(int index)
{
	static const double cx = 325.5;
	static const double cy = 253.5;
	static const double fx = 518.0;
	static const double fy = 519.0;
	static const double depthScale = 1000.0;

	static PointT p ;
	static Eigen::Vector3d point;
	static Eigen::Vector3d pointWorld;

	pointCloud_raw->points.clear();
	color = cv::imread(string(strFilePath + "/" + vstrImageFilenamesRGB[index]), CV_LOAD_IMAGE_UNCHANGED);
	depth = cv::imread(string(strFilePath + "/" + vstrImageFilenamesDepth[index]), CV_LOAD_IMAGE_UNCHANGED);

	for ( size_t v = 0; v < color.rows; v++ )
		for ( size_t  u = 0; u < color.cols; u++ )
		{
			unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
			if ( d == 0 || d > 16000) continue; // 为0表示没有测量到; negelect points too far away

			point[2] = double(d) / depthScale;
			point[0] = (u - cx) * point[2] / fx;
			point[1] = (v - cy) * point[2] / fy;

			pointWorld = T * point;

			p.x = pointWorld[0];
			p.y = -pointWorld[1];
			p.z = -pointWorld[2];

			p.b = color.data[ v * color.step + u * color.channels() ];
			p.g = color.data[ v * color.step + u * color.channels() + 1 ];
			p.r = color.data[ v * color.step + u * color.channels() + 2 ];
			pointCloud_raw->points.push_back( p );
		}

	return 1;
}

void ObjSeg::showCloud(cloudType type)
{
	if (type == RAW)
		viewer->showCloud(pointCloud_raw);
	if (type == FILTERED)
		viewer->showCloud(pointCloud_filtered);
}

void ObjSeg::filtCloud(float leaveSize)
{
	vg.setInputCloud (pointCloud_raw);
	vg.setLeafSize (leaveSize, leaveSize, leaveSize);
	vg.filter (*pointCloud_filtered);
}