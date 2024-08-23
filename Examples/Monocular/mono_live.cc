#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/highgui.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_live path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // Open the camera
    cv::VideoCapture cap(0); // Try 0 first, if it doesn't work, try 1, 2, etc.
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return -1;
    }

    // Set camera properties (adjust as needed)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;
    int proccIm = 0;
    while(true)
    {
        // Read image from camera
        cap >> im;
        if(im.empty())
        {
            cerr << endl << "Failed to grab image from camera!" << endl;
            break;
        }

        proccIm++;

        // Convert to grayscale
        cv::Mat imGray;
        cv::cvtColor(im, imGray, cv::COLOR_BGR2GRAY);

        // Get timestamp
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t1.time_since_epoch()).count();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(imGray, tframe);

        // Display the image (optional)
        /*
        cv::imshow("ORB-SLAM3: Current Frame", imGray);
        if(cv::waitKey(1) == 27) // ESC key
            break;
        */
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}