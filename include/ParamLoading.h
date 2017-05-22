# pragma  once 
#include <string>
#include <vector>

#include <Eigen/Geometry>

using namespace std;

class ParamLoading
{
public:
	const string strFilePath;

	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesDepth;
	vector<double> vTimestamps;
	vector<Eigen::Isometry3d> vCamPoses;
	int nImages;

	ParamLoading(const string strInFilePath);
	void LoadImages();
	void LoadPose();

private:
	string strAssociationFile;
	string strPoseFile;
	string strRGBPath;
	string strDepthPath;
};