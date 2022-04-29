#include <opencv2/opencv.hpp>

#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>

using namespace std;

string parameterFile = "./HuaWeiMatePro30.yaml";
string vocFile = "/home/lighthouse/orb_slam3/ORB_SLAM3/Vocabulary/ORBvoc.bin";

string videoFile = "/home/lighthouse/orb_slam3/testVideo/livingRoom/livingRoom.mp4";
string trajectoryFile = "/home/lighthouse/orb_slam3/testVideo/livingRoom/eating_trajectory.txt";
string camIntrinsicsFile = "/home/lighthouse/orb_slam3/testVideo/livingRoom/camera_intrinsic.txt";

int main(int argc, char **argv) {

// 声明 ORB-SLAM2 系统
ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::MONOCULAR, true);
// 获取视频图像
cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.
// 记录系统时间
auto start = chrono::system_clock::now();
ofstream f, cf;
f.open(trajectoryFile.c_str());
cf.open(camIntrinsicsFile.c_str());
int idx = 0;

while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if (frame.empty())
            break;

        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(720,1280));

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        double imageTimestamp = double(timestamp.count())/1000.0;

        Sophus::SE3f Tcw = SLAM.TrackMonocular(frame_resized, imageTimestamp);
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << imageTimestamp << setprecision(7) << "," << t(0) << "," << t(1) << "," << t(2)
            << "," << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << endl;

        float fx = 503.37802572;
        float fy = 502.14645985;
        float cx = 320.87743973;
        float cy = 236.44062294; 

        cf << setprecision(6) << imageTimestamp << "," << idx << "," << fx << "," << fy << "," << cx << "," << cy << endl;
        idx += 1;
        cv::waitKey(30);
    }
    f.close();
    cf.close();
    SLAM.Shutdown();
    return 0;

}