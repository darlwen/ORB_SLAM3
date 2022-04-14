/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{  
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_myVideo path_to_vocabulary path_to_settings video_file_path " << endl;
        return 1;
    }


    int fps = 20;
    float dT = 1.f/fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);
    float width = SLAM.GetImageWidth();
    float height = SLAM.GetImageHeight();
    
   // cout << "image scale: " << imageScale << endl;

    cv::VideoCapture cap(argv[3]);    // change to 1 if you want to use USB camera.
    // 记录系统时间
    auto start = chrono::system_clock::now();

    double t_resize = 0.f;
    double t_track = 0.f;
    int idx = 0;
    while (1)
    {

        cv::Mat frame;
        cap >> frame;   // 读取相机数据

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        double tframe = double(timestamp.count())/1000.0;

        if(frame.data == nullptr)
        {
            cerr << endl << "Finish loading image at: "
                    <<  tframe << endl;
            break;
        }

        cout << " get one frame image from video" << endl;
        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
          //  int width = frame.cols * imageScale;
          //  int height = frame.rows * imageScale;

          //  cout << "height: " << height << "   width: " << width << endl;

            cv::resize(frame, frame, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(frame,tframe); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            // Wait to load the next frame
            double T=0;
           cv::waitKey(30);

    }
    // Stop all threads
    SLAM.Shutdown();
    cerr << endl << "Finish shut down " << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    cerr << endl << "Finish saving camera trajectory " << endl;


    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    cerr << endl << "Finish saving keyframeTrajectory " << endl;

    return 0;
}


