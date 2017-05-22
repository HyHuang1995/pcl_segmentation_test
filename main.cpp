#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

#include <pcl/filters/passthrough.h>
#include <unistd.h>

#include <ParamLoading.h>
#include <ObjectSegmentation.h>

using namespace std;



void LoadPose(const string &strInFilename, vector<Eigen::Isometry3d> &vCamPoses);

int main(int argc, char** argv)
{
	const string strFilePath = "/home/ardell/WorkSpace/Packages/ORB_SLAM2/rgbd_2_fb3";
	ParamLoading param(strFilePath);
	param.LoadImages();
	param.LoadPose();

	ObjSeg objSeg(param);
	
	for (int ii = 0; ii != param.nImages; ii ++)
	{
		
		objSeg.generateCloud(ii);
		objSeg.filtCloud();
		objSeg.showCloud(objSeg.FILTERED);

		sleep(0.03);
	}

	getchar();

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