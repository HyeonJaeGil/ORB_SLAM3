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

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void normalizeMinMax(cv::Mat &image, double min=-1.0, double max=-1.0);

int main(int argc, char **argv)
{  
    std::string root = "/home/hj/Research/ORB_SLAM3/";
    std::string vocab_file = root + "Vocabulary/ORBvoc.txt";
    std::string settings_file = root + "Examples/Monocular/thermal.yaml";

    vector<string> img_filenames;
    vector<double> timestamps;

    std::string imgs = "/home/hj/Dataset/STHEREO/07/image/thermal_14_left";
    std::string times = "/home/hj/Dataset/STHEREO/07/image/timestamps.txt";

    LoadImages(imgs, times, img_filenames, timestamps);
    cout << "LOADED!" << endl;
    const int nImages = img_filenames.size();

    // Vector for tracking time statistics
    float tracking_times;

    cout << endl << "-------" << endl;
    cout.precision(17);

    // print information
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocab_file, settings_file, ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat im;
    int proccIm = 0;
    for(int ni=0; ni<nImages; ni++, proccIm++)
    {
        // if (ni < 150) continue;

        // Read image from file
        im = cv::imread(img_filenames[ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
        
        // // normalize image into 8 bit with minmax value
        normalizeMinMax(im);
        
        double tframe = timestamps[ni];
        // std::cout << "image: " << img_filenames[ni] <<  ", timestamp: " << tframe << std::endl;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                    <<  img_filenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        // cout << "tframe = " << tframe << endl;
        SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        tracking_times=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = timestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-timestamps[ni-1];

        // std::cout << "T: " << T << std::endl;
        // std::cout << "ttrack: " << ttrack << std::endl;

        if(ttrack<T) {
            //std::cout << "usleep: " << (dT-ttrack) << std::endl;
            usleep((T-ttrack)*1e6); // 1e6
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(15000);
    vstrImages.reserve(15000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}


void normalizeMinMax(cv::Mat &image, double min, double max)
{
    if (min < 0.0 || max < 0.0)
        cv::minMaxLoc(image, &min, &max);

    // clip values with min and max
    cv::Mat mask = image < min;
    image.setTo(min, mask);
    mask = image > max;
    image.setTo(max, mask);

    // normalize
    cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}