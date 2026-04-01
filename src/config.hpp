#pragma once

#include <string>

struct Config {
    std::string tag_family;          // AprilTag family name
    int nthreads;                    // Number of detector threads
    float quad_decimate;             // Image decimation factor
    float quad_sigma;                // Gaussian blur sigma
    bool refine_edges;               // Edge refinement
    float decode_sharpening;         // Decoding sharpening
    int max_error_bits;              // Maximum corrected tag bits
    int pose_refine_iterations;      // Extra OpenCV pose refinement iterations
    double tag_size_meters;          // Physical tag size in meters
    int camera_index;                // Camera index
    double min_decision_margin;      // Minimum confidence threshold
};

inline Config defaultConfig() {
    return Config{
        "tag36h11",
        4,
        2.0f,
        0.0f,
        true,
        0.25f,
        3,
        0,
        0.1651,
        0,
        30.0
    };
}
