#include "ObjectSegmentation.h"

#include <iostream>

ObjSeg::ObjSeg(ParamLoading param):
	strFilePath(param.strFilePath),
	vstrImageFilenamesRGB(param.vstrImageFilenamesRGB),
	vstrImageFilenamesDepth(param.vstrImageFilenamesDepth),
	vCamPoses(param.vCamPoses),
	q(Eigen::Quaterniond(0, 0, 0, 0)), T(Eigen::Isometry3d(q))
{
	T.pretranslate( Eigen::Vector3d( 0, 0, 0 ));

	//initPlaneSeg();
	inliers = pcl::PointIndices::Ptr (new pcl::PointIndices);
	coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.017);

	viewer = new pcl::visualization::CloudViewer("viewer");
	pointCloud_raw = PointCloud::Ptr(new PointCloud) ;
	pointCloud_filtered = PointCloud::Ptr(new PointCloud) ;
	pointCloud_removal = PointCloud::Ptr(new PointCloud) ;
	pointCloud_cluster = PointCloud::Ptr(new PointCloud) ;
	pointCloud_plane = PointCloud::Ptr(new PointCloud);

	tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>);


	cout << endl;
	cout << "Object segmentation module constructed!" << endl;
}

bool ObjSeg::generateCloud(int index)
{
	static const double cx = 325.5;
	static const double cy = 253.5;
	static const double fx = 518.0;
	static const double fy = 519.0;
	static const double depthScale = 5000.0;

	static PointT p ;
	//static Eigen::Isometry3d T;
	static Eigen::Vector3d point;
	static Eigen::Vector3d pointWorld;

	T = vCamPoses[index];

	pointCloud_raw->points.clear();
	color = cv::imread(string(strFilePath + "/" + vstrImageFilenamesRGB[index]), CV_LOAD_IMAGE_UNCHANGED);
	depth = cv::imread(string(strFilePath + "/" + vstrImageFilenamesDepth[index]), CV_LOAD_IMAGE_UNCHANGED);

	// int zmin = 100, zmax = -100;
	for ( size_t v = 0; v < color.rows; v++ )
		for ( size_t  u = 0; u < color.cols; u++ )
		{
			unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
			if ( d == 0 || d > 16000) continue; // 为0表示没有测量到; negelect points too far away

			point[2] = double(d) / depthScale;
			point[0] = (u - cx) * point[2] / fx;
			point[1] = (v - cy) * point[2] / fy;

			pointWorld = T * point;

			p.x = -pointWorld[0];
			p.z = pointWorld[1];
			p.y = pointWorld[2];
			// if (p.z > zmax) zmax = p.z;
			// if (p.z < zmin)zmin = p.z;
			p.b = color.data[ v * color.step + u * color.channels() ];
			p.g = color.data[ v * color.step + u * color.channels() + 1 ];
			p.r = color.data[ v * color.step + u * color.channels() + 2 ];
			pointCloud_raw->points.push_back( p );
		}

	//cout << zmin << ' ' << zmax << endl;

	return 1;
}

void ObjSeg::showCloud(cloudType type)
{
	if (type == RAW)
		viewer->showCloud(pointCloud_raw);
	if (type == FILTERED)
	{
		//cout << "view filtered cloud" << endl;
		viewer->showCloud(pointCloud_filtered);
	}
	if (type == REMOVAL)
		viewer->showCloud(pointCloud_removal);
}

void ObjSeg::filtCloud(float leaveSize)
{
	//voxelgrid filter
	vg.setInputCloud (pointCloud_raw);
	vg.setLeafSize (leaveSize, leaveSize, leaveSize);
	vg.filter (*pointCloud_filtered);

	//pass
	pass.setInputCloud(pointCloud_filtered);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.1, 10);
	pass.filter(*pointCloud_filtered);

	pass.setInputCloud(pointCloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-2.55, 10);

	pass.filter(*pointCloud_filtered);
}

void ObjSeg::removePlane()
{
	int nr_points = pointCloud_filtered->points.size();
	//*pointCloud_removal = *pointCloud_filtered;
	viewer->showCloud(pointCloud_removal);
	sleep(1);
	cout << "nrpoints" << nr_points << endl;
	do//while (pointCloud_filtered->points.size () > 0.5 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		//cout << "cloudsize" << pointCloud_filtered->points.size () << endl;
		seg.setInputCloud (pointCloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			//break;
		}

		// Extract the planar inliers from the input cloud

		extract.setInputCloud (pointCloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*pointCloud_plane);
		//std::cout << "PointCloud representing the planar component: " << pointCloud_plane->points.size () << " data points." << std::endl;

		//viewer->showCloud( pointCloud_plane );
		//pointCloud->points.clear();
		//sleep(1);

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*pointCloud_removal);
		*pointCloud_filtered = *pointCloud_removal;
	} while (pointCloud_plane->points.size () > 2500);

}

void ObjSeg::cluster()
{
	tree->setInputCloud (pointCloud_filtered);
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (10000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pointCloud_filtered);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		// unsigned char r = rand() / 128;
		// unsigned char g = rand() / 128;
		// unsigned char b = rand() / 128;

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			// cloud_filtered->points[*pit].r = r;
			// cloud_filtered->points[*pit].g = g;
			// cloud_filtered->points[*pit].b = b;
			pointCloud_cluster->points.push_back (pointCloud_filtered->points[*pit]); //*
		}
		//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	}
	pointCloud_cluster-> is_dense = true;

	//viewer.showCloud( cloud_cluster );
}

void ObjSeg::initPlaneSeg()
{
	inliers = pcl::PointIndices::Ptr (new pcl::PointIndices);
	coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.014);
}
