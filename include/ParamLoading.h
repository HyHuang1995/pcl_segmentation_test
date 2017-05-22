# pragma  once 
#include <string>
#include <vector>

using namespace std;

class ParamLoading
{
public:
	const string strFilePath;

	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesDepth;
	vector<double> vTimestamps;
	int nImages;

	ParamLoading(const string strInFilePath);
	void LoadImages();
	void LoadPose();

private:
	string strAssociationFile;
	string strRGBPath;
	string strDepthPath;
};