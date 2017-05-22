#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <unistd.h>

using namespace std;


void LoadImages(const string &strInFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadPose(const string &strInFilename, vector<Eigen::Isometry3d> &vCamPoses);

int main(int argc, char** argv)
{
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<double> vTimestamps;

	vector<Eigen::Isometry3d> vCamPoses;

	string strinFilename = string(argv[1]);
	string strPoseFileName = string(argv[2]);
	//cout << argv[0] << ' ' << argv[1] << endl;
	LoadImages(strinFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
	LoadPose(strPoseFileName, vCamPoses);

	cout << "load successfully!" << endl;
	// 相机内参
	pcl::visualization::CloudViewer viewer("viewer");

	double cx = 325.5;
	double cy = 253.5;
	double fx = 518.0;
	double fy = 519.0;
	double depthScale = 1000.0;

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

	// 新建一个点云
	PointCloud::Ptr pointCloud( new PointCloud );
	pointCloud->is_dense = false;

	// pass through
	PointCloud::Ptr pointCloud_passthrough( new PointCloud );
	pcl::PassThrough<PointT> pass;

	int nImages = vstrImageFilenamesRGB.size();
	cv::Mat color, depth;

	cout << "load successfully!" << endl;
	cout << vCamPoses.size() << endl;

	//Eigen::Quaterniond q( 1, 0, 0, 0 );
	//Eigen::Isometry3d T(q);
	//T.pretranslate( Eigen::Vector3d( 0, 0, 0 ));
	PointT p ;
	Eigen::Vector3d point;
	Eigen::Vector3d pointWorld;
	//Eigen::Isometry3d T;

	Eigen::Quaterniond q( 1, 0, 0, 0 );
	Eigen::Isometry3d T(q);
	T.pretranslate( Eigen::Vector3d( 0, 0, 0 ));
	vCamPoses.push_back( T );
	for (int ii = 0; ii != nImages; ii ++)
	{
		//T = vCamPoses[ii];
		color = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[ii], CV_LOAD_IMAGE_UNCHANGED);
		depth = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesD[ii], CV_LOAD_IMAGE_UNCHANGED);

		double xmax = 0, xmin = 0, zmax = 0;
		for ( int v = 0; v < color.rows; v++ )
			for ( int u = 0; u < color.cols; u++ )
			{
				unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
				if ( d == 0 || d > 16000) continue; // 为0表示没有测量到

				point[2] = double(d) / depthScale;
				point[0] = (u - cx) * point[2] / fx;
				point[1] = (v - cy) * point[2] / fy;

				pointWorld = T * point;

				p.x = pointWorld[0];
				p.y = -pointWorld[1];
				p.z = -pointWorld[2];
				// if (d > xmax)
				// 	xmax = d;
				// if (p.x < xmin)
				// 	xmin = p.x;
				
				p.b = color.data[ v * color.step + u * color.channels() ];
				p.g = color.data[ v * color.step + u * color.channels() + 1 ];
				p.r = color.data[ v * color.step + u * color.channels() + 2 ];
				pointCloud->points.push_back( p );
			}
		//cout << xmax << ' '<< xmin << ' ' << zmax << endl;

		// pass.setInputCloud (pointCloud);
		// pass.setFilterFieldName ("x");
		// pass.setFilterLimits (-10.0, 10.0);
		// //pass.setFilterLimitsNegative (true);
		// pass.filter (*pointCloud_passthrough);

		// pass.setInputCloud (pointCloud_passthrough);
		// pass.setFilterFieldName ("y");
		// pass.setFilterLimits (-12.0, 12.0);
		// //pass.setFilterLimitsNegative (true);
		// pass.filter (*pointCloud_passthrough);


		viewer.showCloud( pointCloud );
		pointCloud->points.clear();
		sleep(0.03);
	}

	getchar();

}

void LoadImages(const string &strInFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
	ifstream fin;
	fin.open(strInFilename.c_str());
	while (!fin.eof())
	{
		string s;
		getline(fin, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB, sD;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenamesRGB.push_back(sRGB);
			ss >> t;
			ss >> sD;
			vstrImageFilenamesD.push_back(sD);

		}
	}
}

void LoadPose(const string &strFilename, vector<Eigen::Isometry3d> &vCamPoses)
{
	ifstream fin;
	fin.open(strFilename.c_str());
	while (!fin.eof())
	{
		string s;
		getline(fin, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double tmp;
			for (size_t ii = 0; ii != 4; ii ++)
				ss >> tmp;
			double qx, qy, qz, qw;
			ss >> qx;
			ss >> qy;
			ss >> qz;
			ss >> qw;

			Eigen::Quaterniond q( qw, qx, qy, 0 );
			Eigen::Isometry3d T(q);
			T.pretranslate( Eigen::Vector3d( 0, 0, 0 ));
			vCamPoses.push_back( T );

		}
		//getline(fin, s);
		//getline(fin, s);
	}
}