#include <opencv2/opencv.hpp>
#include <string>
#include <chrono> // for time stamp
#include <vector>
#include <fstream>

#include<iostream>
#include<System.h>
#include<algorithm>



using namespace std;
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./scan_7.yaml";
string vocFile = "./Vocabulary/ORBvoc.txt";
// 视频文件，若不同请修改
// string videoFile = "/home/guojun/dataset/scan_0.mp4";
string root = "/home/guojun/dataset/scan_0.mp4";
int video_frame = 0;




int main(int argc, char **argv)
{

    ifstream f_test;
    string strPathTimeFile = "/home/guojun/dataset/test.txt";
    f_test.open(strPathTimeFile.c_str());
    while (!f_test.eof())
    {
        string s;
        getline(f_test, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            cout<<ss.str()<<endl;
            double t;
            ss >> t;
            cout<< t <<endl;
        }
    }



    return 0;
}
