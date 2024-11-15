#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// Function to deproject depth frame into 3D points
std::vector<Eigen::Vector3f> deproject_depth_to_3d(rs2::depth_frame depth_frame, rs2_intrinsics intrinsics) {
    std::vector<Eigen::Vector3f> points;

    for (int y = 0; y < depth_frame.get_height(); ++y) {
        for (int x = 0; x < depth_frame.get_width(); ++x) {
            float depth = depth_frame.get_distance(x, y);
            if (depth > 0) {  // Ignore invalid depth
                float point[3];
                float pixel[2] = { static_cast<float>(x), static_cast<float>(y) };
                rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
                points.emplace_back(point[0], point[1], point[2]);
            }
        }
    }
    return points;
}

// Compute the tilt angle about the Z-axis
float compute_tilt_angle(const std::vector<Eigen::Vector3f>& points1, const std::vector<Eigen::Vector3f>& points2) {
    // Center points around their centroids
    Eigen::Vector3f centroid1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid2 = Eigen::Vector3f::Zero();

    for (const auto& p : points1) centroid1 += p;
    for (const auto& p : points2) centroid2 += p;

    centroid1 /= points1.size();
    centroid2 /= points2.size();

    std::vector<Eigen::Vector3f> centered1, centered2;
    for (const auto& p : points1) centered1.push_back(p - centroid1);
    for (const auto& p : points2) centered2.push_back(p - centroid2);

    // Compute cross-covariance matrix
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < centered1.size(); ++i) {
        H += centered1[i] * centered2[i].transpose();
    }

    // Compute SVD of H
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    // Extract tilt angle about Z-axis
    float tilt_angle = atan2(R(1, 0), R(0, 0));
    return tilt_angle;
}

int main() {
    // Initialize RealSense pipeline
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipeline.start(config);

    try {
        // Capture two depth frames
        std::cout << "Capture the first depth frame and rotate the camera.\n";
        rs2::frameset frames1 = pipeline.wait_for_frames();
        rs2::depth_frame depth_frame1 = frames1.get_depth_frame();

        std::cout << "Rotate the camera and capture the second depth frame.\n";
        rs2::frameset frames2 = pipeline.wait_for_frames();
        rs2::depth_frame depth_frame2 = frames2.get_depth_frame();

            // Get depth intrinsics
            rs2_intrinsics intrinsics = depth_frame1.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        // Deproject depth frames into 3D points
        auto points1 = deproject_depth_to_3d(depth_frame1, intrinsics);
        auto points2 = deproject_depth_to_3d(depth_frame2, intrinsics);

        // Compute tilt angle
        float tilt_angle = compute_tilt_angle(points1, points2);
        std::cout << "Tilt angle about Z-axis: " << tilt_angle * 180.0 / M_PI << " degrees" << std::endl;

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "General error: " << e.what() << std::endl;
    }

    // Stop the camera
    pipeline.stop();
    return 0;
}