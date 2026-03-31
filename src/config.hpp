#pragma once

struct Config {
    int nthreads;                    // Number of detector threads
    float quad_decimate;             // Image decimation factor
    float quad_sigma;                // Gaussian blur sigma
    bool refine_edges;               // Edge refinement
    float decode_sharpening;         // Decoding sharpening
    double tag_size_meters;          // Physical tag size in meters
    int camera_index;                // Camera index
    double min_decision_margin;      // Minimum confidence threshold
};

inline Config defaultConfig() {
    return Config{
        4,
        2.0f,
        0.0f,
        true,
        0.25f,
        0.1651,
        0,
        50.0
    };
}
