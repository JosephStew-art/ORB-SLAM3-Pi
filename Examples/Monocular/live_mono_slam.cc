#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <deque>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

void lockCameraAttributes(cv::VideoCapture& cap) {
    // ... (existing code remains unchanged)
}

int main(int argc, char **argv) {
    if(argc != 3) {
        cerr << endl << "Usage: ./live_mono_slam path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Set up signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // Initialize webcam
    cv::VideoCapture cap(0); // Open default camera (usually the webcam)
    if (!cap.isOpened()) {
        cerr << "Error: Couldn't open the camera." << endl;
        return -1;
    }

    // Set camera parameters
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    // Lock camera attributes
    //lockCameraAttributes(cap);

    cv::Mat frame;
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    // Processing time calculation variables
    const int WINDOW_SIZE = 30;  // Calculate average over 30 frames
    std::deque<double> processing_times;
    double total_processing_time = 0.0;

    while(b_continue_session) {
        // Capture frame
        cap >> frame;
        if (frame.empty()) {
            cerr << "Error: Blank frame grabbed" << endl;
            break;
        }

        // Get timestamp
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0;

        // Measure SLAM processing time
        auto slam_start = std::chrono::high_resolution_clock::now();
        
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, timestamp);
        
        auto slam_end = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration<double, std::milli>(slam_end - slam_start).count();

        // Update processing times
        processing_times.push_back(processing_time);
        total_processing_time += processing_time;

        if (processing_times.size() > WINDOW_SIZE) {
            total_processing_time -= processing_times.front();
            processing_times.pop_front();
        }

        frame_count++;

        // Print average processing time and effective FPS every WINDOW_SIZE frames
        if (frame_count % WINDOW_SIZE == 0) {
            double avg_processing_time = total_processing_time / WINDOW_SIZE;
            double effective_fps = 1000.0 / avg_processing_time;  // Convert ms to seconds

            cout << "Average processing time: " << std::fixed << std::setprecision(2) << avg_processing_time << " ms" << endl;
            cout << "Effective FPS: " << std::fixed << std::setprecision(2) << effective_fps << endl;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SavePointCloud("pointcloud.txt");

    return 0;
}