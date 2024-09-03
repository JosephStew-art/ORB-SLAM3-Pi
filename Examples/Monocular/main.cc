#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include "System.h"

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: ./mono_webcam path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // Open the camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the camera." << std::endl;
        return 1;
    }

    // Set camera properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    cv::Mat frame;
    double t_frame = 0.0;
    while (true)
    {
        // Capture frame
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Error: Could not capture frame." << std::endl;
            break;
        }

        // Convert to RGB if necessary
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        // Resize frame
        cv::resize(frame, frame, cv::Size(600, 350));

        // Get timestamp
        t_frame = cv::getTickCount() / cv::getTickFrequency();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, t_frame);

        // Display the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(30) == 27) // Wait for 30ms or until 'Esc' is pressed
        { 
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}