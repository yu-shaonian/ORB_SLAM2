#include <opencv2/opencv.hpp>
#include "System.h"
#include <string>
#include <chrono> // for time stamp
#include <iostream>
#include "Converter.h"

//#include <io.h>
#include <string>
#include <vector>
#include <fstream>


using namespace std;
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./scan_7.yaml";
string vocFile = "./Vocabulary/ORBvoc.txt";
// 视频文件，若不同请修改
// string videoFile = "/home/guojun/dataset/scan_0.mp4";
string root = "/home/guojun/dataset/scan_0.mp4";
int video_frame = 0;

/*
path: 指定目录
files: 保存结果
fileType: 指定的文件格式，如 .jpg
*/



void LoadImages(vector<string> &vstrImageFilenames, vector<double> &vTimeStamps, const string &imgPathFile)
{
    ifstream f_test;
    f_test.open(imgPathFile.c_str());
    while (!f_test.eof())
    {
        string s;
        getline(f_test, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vstrImageFilenames.push_back("/home/guojun/dataset/scene0000_00/color/" + ss.str() + ".jpg");
            vTimeStamps.push_back(t);

        }
    }
}

int main(int argc, char **argv)
{
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);
    // 获取视频图像
    auto start = chrono::system_clock::now();

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

//    ifstream f_test;
    string imgPathFile = "/home/guojun/dataset/scene0000.txt";
    LoadImages(vstrImageFilenames, vTimestamps, imgPathFile);

    int nImages = vTimestamps.size();

    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        SLAM.TrackMonocular(im, tframe);
        cv::waitKey(30);
    }

    // Stop all threads
    SLAM.Shutdown();
    auto frame_all_pose = SLAM.mpTracker->mlRelativeFramePoses;

    cout << "目前有效帧数：" << frame_all_pose.size() << endl;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("./result/scan_0.txt");
    SLAM.SaveAllFrame("./result/scan_all_0.txt");

    return 0;
}
