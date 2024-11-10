#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>    // Include OpenCV for image handling
#include <iostream>
#include <fstream>

int main() {
    try {
        // Declare RealSense pipeline
        rs2::pipeline pipe;
        pipe.start(); // Start streaming

        // Wait for a depth frame
        auto frames = pipe.wait_for_frames();
        auto depth_frame = frames.get_depth_frame();

        // Get dimensions of the depth frame
        const int width = depth_frame.get_width();
        const int height = depth_frame.get_height();

        // Create an OpenCV matrix to store depth data
        cv::Mat depth_image(height, width, CV_16UC1);

        // Populate the matrix with depth data in millimeters
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // Get the depth value at (x, y) in meters and convert to millimeters
                uint16_t depth_value = static_cast<uint16_t>(depth_frame.get_distance(x, y) * 1000);
                depth_image.at<uint16_t>(y, x) = depth_value;
            }
        }

        // Save the depth image as a PNG file
        cv::imwrite("depth_image.png", depth_image);
        std::cout << "Depth image saved as depth_image.png" << std::endl;

        // Save the depth data to a CSV file
        std::ofstream csv_file("depth_data.csv");
        if (csv_file.is_open()) {
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    csv_file << depth_image.at<uint16_t>(y, x);
                    if (x < width - 1) csv_file << ",";
                }
                csv_file << "\n";
            }
            csv_file.close();
            std::cout << "Depth data saved as depth_data.csv" << std::endl;
        } else {
            std::cerr << "Unable to open CSV file for writing." << std::endl;
        }

        pipe.stop(); // Stop the pipeline
    } catch (const rs2::error &e) {
        std::cerr << "RealSense error calling " << e.get_failed_function()
                  << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
