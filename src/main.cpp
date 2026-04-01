#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include "calibration.hpp"
#include "config.hpp"
#include "detector.hpp"
#include "field_layout.hpp"
#include "pose.hpp"
#include "publisher.hpp"
#include "visualize.hpp"

#if HAVE_NTCORE
#include <cameraserver/CameraServer.h>
#include <cscore_cv.h>
#endif

#if HAVE_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
std::atomic<bool> g_stop{false};
using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

enum class OutputOrientation {
    Normal,
    Rotate90Clockwise,
    Rotate180,
    Rotate90CounterClockwise
};

void onSigInt(int) {
    g_stop.store(true);
}

struct CliOptions {
    int camera_index = 0;
    double tag_size_meters = 0.1651;
    std::string calibration_path;
    std::string camera_name = "camera0";
    bool no_display = false;
    bool stdout_json = false;
    std::optional<std::string> record_path;
    std::optional<unsigned int> team_number;
    std::optional<std::string> nt_server;
    std::optional<int> width;
    std::optional<int> height;
    std::optional<int> fps;
    std::optional<int> threads;
    std::optional<float> quad_decimate;
    std::optional<float> quad_sigma;
    std::optional<bool> refine_edges;
    std::optional<int> pose_iterations;
    std::optional<int> max_error_bits;
    std::optional<double> decision_margin;
    std::optional<std::string> tag_family;
    std::optional<bool> auto_exposure;
    std::optional<double> exposure;
    std::optional<double> brightness;
    std::optional<bool> auto_white_balance;
    std::optional<double> white_balance;
    std::optional<OutputOrientation> orientation;
    std::optional<int> stream_width;
    std::optional<int> stream_height;
    std::optional<std::string> field_layout_path;
};

int parseInteger(const std::string& value, const std::string& flag) {
    try {
        return std::stoi(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid integer for " + flag + ": " + value);
    }
}

unsigned int parseUnsigned(const std::string& value, const std::string& flag) {
    unsigned int parsed = 0U;
    try {
        parsed = static_cast<unsigned int>(std::stoul(value));
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid unsigned integer for " + flag + ": " + value);
    }

    if (parsed == 0U) {
        throw std::runtime_error(flag + " must be greater than zero");
    }
    return parsed;
}

double parseDouble(const std::string& value, const std::string& flag) {
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid number for " + flag + ": " + value);
    }
}

float parseFloat(const std::string& value, const std::string& flag) {
    try {
        return std::stof(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid number for " + flag + ": " + value);
    }
}

int parsePositiveInt(const std::string& value, const std::string& flag) {
    const int parsed = parseInteger(value, flag);
    if (parsed <= 0) {
        throw std::runtime_error(flag + " must be greater than zero");
    }
    return parsed;
}

double parseNonNegativeDouble(const std::string& value, const std::string& flag) {
    const double parsed = parseDouble(value, flag);
    if (parsed < 0.0) {
        throw std::runtime_error(flag + " must be non-negative");
    }
    return parsed;
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool parseOnOff(const std::string& value, const std::string& flag) {
    const std::string lowered = toLower(value);
    if (lowered == "on" || lowered == "true" || lowered == "1") {
        return true;
    }
    if (lowered == "off" || lowered == "false" || lowered == "0") {
        return false;
    }
    throw std::runtime_error("Invalid value for " + flag + ": " + value + " (expected on/off)");
}

std::string normalizeTagFamilyName(const std::string& value) {
    const std::string lowered = toLower(value);
    if (lowered == "16h5" || lowered == "tag16h5") {
        return "tag16h5";
    }
    if (lowered == "25h9" || lowered == "tag25h9") {
        return "tag25h9";
    }
    if (lowered == "36h10" || lowered == "tag36h10") {
        return "tag36h10";
    }
    if (lowered == "36h11" || lowered == "tag36h11") {
        return "tag36h11";
    }
    if (lowered == "circle21h7" || lowered == "tagcircle21h7") {
        return "tagCircle21h7";
    }
    if (lowered == "circle49h12" || lowered == "tagcircle49h12") {
        return "tagCircle49h12";
    }
    if (lowered == "custom48h12" || lowered == "tagcustom48h12") {
        return "tagCustom48h12";
    }
    if (lowered == "standard41h12" || lowered == "tagstandard41h12") {
        return "tagStandard41h12";
    }
    if (lowered == "standard52h13" || lowered == "tagstandard52h13") {
        return "tagStandard52h13";
    }

    throw std::runtime_error(
        "Unsupported --family value: " + value +
        ". Supported families: tag16h5, tag25h9, tag36h10, tag36h11, "
        "tagCircle21h7, tagCircle49h12, tagCustom48h12, tagStandard41h12, tagStandard52h13");
}

int defaultMaxErrorBitsForFamily(const std::string& family) {
    const std::string normalized = normalizeTagFamilyName(family);
    if (normalized == "tag16h5") {
        return 0;
    }
    if (normalized == "tag36h11") {
        return 3;
    }
    return 2;
}

OutputOrientation parseOrientation(const std::string& value) {
    const std::string lowered = toLower(value);
    if (lowered == "normal" || lowered == "0") {
        return OutputOrientation::Normal;
    }
    if (lowered == "cw90" || lowered == "90" || lowered == "rotate90" || lowered == "rotate90cw") {
        return OutputOrientation::Rotate90Clockwise;
    }
    if (lowered == "180" || lowered == "rotate180" || lowered == "flip") {
        return OutputOrientation::Rotate180;
    }
    if (lowered == "ccw90" || lowered == "270" || lowered == "rotate270" || lowered == "rotate90ccw") {
        return OutputOrientation::Rotate90CounterClockwise;
    }

    throw std::runtime_error(
        "Invalid value for --orientation: " + value +
        " (expected normal, cw90, 180, or ccw90)");
}

std::string orientationName(OutputOrientation orientation) {
    switch (orientation) {
        case OutputOrientation::Normal:
            return "normal";
        case OutputOrientation::Rotate90Clockwise:
            return "cw90";
        case OutputOrientation::Rotate180:
            return "180";
        case OutputOrientation::Rotate90CounterClockwise:
            return "ccw90";
    }
    return "normal";
}

bool trySetCameraProperty(cv::VideoCapture& cap, int property, const std::vector<double>& candidates) {
    for (double candidate : candidates) {
        if (cap.set(property, candidate)) {
            return true;
        }
    }
    return false;
}

void applyOutputOrientation(const cv::Mat& input, cv::Mat& output, OutputOrientation orientation) {
    switch (orientation) {
        case OutputOrientation::Normal:
            output = input;
            return;
        case OutputOrientation::Rotate90Clockwise:
            cv::rotate(input, output, cv::ROTATE_90_CLOCKWISE);
            return;
        case OutputOrientation::Rotate180:
            cv::rotate(input, output, cv::ROTATE_180);
            return;
        case OutputOrientation::Rotate90CounterClockwise:
            cv::rotate(input, output, cv::ROTATE_90_COUNTERCLOCKWISE);
            return;
    }
}

void applyCameraControls(cv::VideoCapture& cap, const CliOptions& cli) {
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (cli.width.has_value()) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, *cli.width);
    }
    if (cli.height.has_value()) {
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, *cli.height);
    }
    if (cli.fps.has_value()) {
        cap.set(cv::CAP_PROP_FPS, *cli.fps);
    }
    if (cli.auto_exposure.has_value()) {
        const bool enabled = *cli.auto_exposure;
        const bool applied = enabled
                                 ? trySetCameraProperty(cap, cv::CAP_PROP_AUTO_EXPOSURE, {0.75, 1.0, 3.0})
                                 : trySetCameraProperty(cap, cv::CAP_PROP_AUTO_EXPOSURE, {0.25, 0.0, 1.0});
        if (!applied) {
            std::cerr << "Warning: failed to set auto exposure on this camera/backend.\n";
        }
    }
    if (cli.exposure.has_value()) {
        if (!cap.set(cv::CAP_PROP_EXPOSURE, *cli.exposure)) {
            std::cerr << "Warning: failed to set manual exposure on this camera/backend.\n";
        }
    }
    if (cli.brightness.has_value()) {
        if (!cap.set(cv::CAP_PROP_BRIGHTNESS, *cli.brightness)) {
            std::cerr << "Warning: failed to set brightness on this camera/backend.\n";
        }
    }
    if (cli.auto_white_balance.has_value()) {
        if (!cap.set(cv::CAP_PROP_AUTO_WB, *cli.auto_white_balance ? 1.0 : 0.0)) {
            std::cerr << "Warning: failed to set auto white balance on this camera/backend.\n";
        }
    }
    if (cli.white_balance.has_value()) {
        if (!cap.set(cv::CAP_PROP_WB_TEMPERATURE, *cli.white_balance)) {
            std::cerr << "Warning: failed to set white balance temperature on this camera/backend.\n";
        }
    }
}

std::int64_t nowUnixMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               SystemClock::now().time_since_epoch())
        .count();
}

bool readFrame(cv::VideoCapture& cap, cv::Mat& frame, std::int64_t& timestamp_us) {
    if (!cap.read(frame) || frame.empty()) {
        return false;
    }

    timestamp_us = nowUnixMicros();
    return true;
}

class FramePreprocessor {
public:
    FramePreprocessor() {
#if HAVE_OPENCV_CUDA
        try {
            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                cv::cuda::setDevice(0);
                using_cuda_ = true;
                backend_ = "cuda";
            }
        } catch (const cv::Exception&) {
            using_cuda_ = false;
            backend_ = "cpu";
        }
#endif
    }

    const std::string& backend() const {
        return backend_;
    }

    void toGray(const cv::Mat& frame, cv::Mat& gray) {
#if HAVE_OPENCV_CUDA
        if (using_cuda_) {
            try {
                gpu_frame_.upload(frame);
                cv::cuda::cvtColor(gpu_frame_, gpu_gray_, cv::COLOR_BGR2GRAY);
                gpu_gray_.download(gray);
                return;
            } catch (const cv::Exception&) {
                using_cuda_ = false;
                backend_ = "cpu";
            }
        }
#endif
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

private:
    bool using_cuda_ = false;
    std::string backend_ = "cpu";
#if HAVE_OPENCV_CUDA
    cv::cuda::GpuMat gpu_frame_;
    cv::cuda::GpuMat gpu_gray_;
#endif
};

CliOptions parseArgs(int argc, char** argv) {
    CliOptions opts;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto requireValue = [&](const std::string& flag) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            return argv[++i];
        };

        if (arg == "--camera") {
            opts.camera_index = parseInteger(requireValue("--camera"), "--camera");
        } else if (arg == "--tag-size") {
            opts.tag_size_meters = parseDouble(requireValue("--tag-size"), "--tag-size");
        } else if (arg == "--calibration") {
            opts.calibration_path = requireValue("--calibration");
        } else if (arg == "--camera-name") {
            opts.camera_name = requireValue("--camera-name");
        } else if (arg == "--no-display") {
            opts.no_display = true;
        } else if (arg == "--stdout-json") {
            opts.stdout_json = true;
        } else if (arg == "--record") {
            opts.record_path = requireValue("--record");
        } else if (arg == "--team") {
            opts.team_number = parseUnsigned(requireValue("--team"), "--team");
        } else if (arg == "--nt-server") {
            opts.nt_server = requireValue("--nt-server");
        } else if (arg == "--width") {
            opts.width = parsePositiveInt(requireValue("--width"), "--width");
        } else if (arg == "--height") {
            opts.height = parsePositiveInt(requireValue("--height"), "--height");
        } else if (arg == "--fps") {
            opts.fps = parsePositiveInt(requireValue("--fps"), "--fps");
        } else if (arg == "--threads") {
            opts.threads = parsePositiveInt(requireValue("--threads"), "--threads");
        } else if (arg == "--quad-decimate") {
            const float quad_decimate = parseFloat(requireValue("--quad-decimate"), "--quad-decimate");
            if (quad_decimate <= 0.0f) {
                throw std::runtime_error("--quad-decimate must be greater than zero");
            }
            opts.quad_decimate = quad_decimate;
        } else if (arg == "--blur" || arg == "--quad-sigma") {
            opts.quad_sigma = static_cast<float>(
                parseNonNegativeDouble(requireValue(arg), arg));
        } else if (arg == "--refine-edges") {
            opts.refine_edges = parseOnOff(requireValue("--refine-edges"), "--refine-edges");
        } else if (arg == "--pose-iterations") {
            const int pose_iterations = parseInteger(requireValue("--pose-iterations"), "--pose-iterations");
            if (pose_iterations < 0 || pose_iterations > 100) {
                throw std::runtime_error("--pose-iterations must be in the range [0, 100]");
            }
            opts.pose_iterations = pose_iterations;
        } else if (arg == "--max-error-bits") {
            const int max_error_bits = parseInteger(requireValue("--max-error-bits"), "--max-error-bits");
            if (max_error_bits < 0) {
                throw std::runtime_error("--max-error-bits must be non-negative");
            }
            opts.max_error_bits = max_error_bits;
        } else if (arg == "--decision-margin") {
            opts.decision_margin =
                parseNonNegativeDouble(requireValue("--decision-margin"), "--decision-margin");
        } else if (arg == "--decision-margin-cutoff") {
            opts.decision_margin =
                parseNonNegativeDouble(requireValue("--decision-margin-cutoff"), "--decision-margin-cutoff");
        } else if (arg == "--family") {
            opts.tag_family = normalizeTagFamilyName(requireValue("--family"));
        } else if (arg == "--auto-exposure") {
            opts.auto_exposure = parseOnOff(requireValue("--auto-exposure"), "--auto-exposure");
        } else if (arg == "--exposure") {
            opts.exposure = parseDouble(requireValue("--exposure"), "--exposure");
        } else if (arg == "--brightness") {
            opts.brightness = parseDouble(requireValue("--brightness"), "--brightness");
        } else if (arg == "--auto-white-balance") {
            opts.auto_white_balance =
                parseOnOff(requireValue("--auto-white-balance"), "--auto-white-balance");
        } else if (arg == "--white-balance") {
            opts.white_balance =
                parseNonNegativeDouble(requireValue("--white-balance"), "--white-balance");
        } else if (arg == "--orientation") {
            opts.orientation = parseOrientation(requireValue("--orientation"));
        } else if (arg == "--stream-width") {
            opts.stream_width = parsePositiveInt(requireValue("--stream-width"), "--stream-width");
        } else if (arg == "--stream-height") {
            opts.stream_height = parsePositiveInt(requireValue("--stream-height"), "--stream-height");
        } else if (arg == "--field-layout") {
            opts.field_layout_path = requireValue("--field-layout");
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (opts.calibration_path.empty()) {
        throw std::runtime_error("Missing required argument: --calibration <path>");
    }
    if (opts.tag_size_meters <= 0.0) {
        throw std::runtime_error("--tag-size must be greater than zero");
    }
    if (opts.camera_name.empty()) {
        throw std::runtime_error("--camera-name must not be empty");
    }
    if (opts.nt_server.has_value() && opts.nt_server->empty()) {
        throw std::runtime_error("--nt-server must not be empty");
    }
    if (opts.field_layout_path.has_value() && opts.field_layout_path->empty()) {
        throw std::runtime_error("--field-layout must not be empty");
    }
    if (opts.exposure.has_value() && (!opts.auto_exposure.has_value() || *opts.auto_exposure)) {
        throw std::runtime_error("--exposure requires --auto-exposure off");
    }
    if (opts.white_balance.has_value() &&
        (!opts.auto_white_balance.has_value() || *opts.auto_white_balance)) {
        throw std::runtime_error("--white-balance requires --auto-white-balance off");
    }
    if (opts.stream_width.has_value() != opts.stream_height.has_value()) {
        throw std::runtime_error("--stream-width and --stream-height must be provided together");
    }

    return opts;
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSigInt);

    try {
        CliOptions cli = parseArgs(argc, argv);

        Config config = defaultConfig();
        config.camera_index = cli.camera_index;
        config.tag_size_meters = cli.tag_size_meters;
        if (cli.tag_family.has_value()) {
            config.tag_family = *cli.tag_family;
        }
        if (cli.threads.has_value()) {
            config.nthreads = *cli.threads;
        }
        if (cli.quad_decimate.has_value()) {
            config.quad_decimate = *cli.quad_decimate;
        }
        if (cli.quad_sigma.has_value()) {
            config.quad_sigma = *cli.quad_sigma;
        }
        if (cli.refine_edges.has_value()) {
            config.refine_edges = *cli.refine_edges;
        }
        if (cli.pose_iterations.has_value()) {
            config.pose_refine_iterations = *cli.pose_iterations;
        }
        config.max_error_bits = cli.max_error_bits.value_or(defaultMaxErrorBitsForFamily(config.tag_family));
        if (cli.decision_margin.has_value()) {
            config.min_decision_margin = *cli.decision_margin;
        }
        if (config.tag_family == "tag16h5" && config.max_error_bits > 0) {
            std::cerr << "Warning: tag16h5 is typically used with --max-error-bits 0.\n";
        }
        if (config.tag_family == "tag36h11" && config.max_error_bits > 3) {
            std::cerr << "Warning: 36h11 is typically used with --max-error-bits 3 or less.\n";
        }
        const OutputOrientation output_orientation =
            cli.orientation.value_or(OutputOrientation::Normal);

        cv::Mat camera_matrix, dist_coeffs;
        try {
            loadCalibration(cli.calibration_path, camera_matrix, dist_coeffs);
        } catch (const std::exception& e) {
            std::cerr << "Calibration error: " << e.what() << '\n';
            return 1;
        }

        std::optional<FieldLayout> field_layout;
        if (cli.field_layout_path.has_value()) {
            try {
                field_layout = loadFieldLayout(*cli.field_layout_path);
                std::cerr << "Loaded field layout with " << field_layout->tags_by_id.size() << " tags";
                if (field_layout->field_length_m.has_value() && field_layout->field_width_m.has_value()) {
                    std::cerr << " (" << *field_layout->field_length_m << "m x "
                              << *field_layout->field_width_m << "m)";
                }
                std::cerr << '\n';
            } catch (const std::exception& e) {
                std::cerr << "Field layout error: " << e.what() << '\n';
                return 1;
            }
        }

        cv::VideoCapture cap(config.camera_index);
        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera index " << config.camera_index << '\n';
            return 1;
        }

        applyCameraControls(cap, cli);

        cv::Mat frame;
        cv::Mat output_frame;
        cv::Mat stream_frame;
        std::int64_t frame_timestamp_us = 0;
        if (!readFrame(cap, frame, frame_timestamp_us)) {
            std::cerr << "Camera frame read failed.\n";
            return 1;
        }

        FramePreprocessor preprocessor;

        const double actual_fps = cap.get(cv::CAP_PROP_FPS);
        std::cerr << "Camera mode: " << frame.cols << 'x' << frame.rows;
        if (actual_fps > 1.0) {
            std::cerr << " @ " << actual_fps << " FPS";
        }
        std::cerr << '\n';
        std::cerr << "AprilTag family: " << config.tag_family << '\n';
        std::cerr << "AprilTag tuning: decimate=" << config.quad_decimate
                  << ", blur=" << config.quad_sigma
                  << ", threads=" << config.nthreads
                  << ", refine_edges=" << (config.refine_edges ? "on" : "off")
                  << ", pose_iterations=" << config.pose_refine_iterations
                  << ", max_error_bits=" << config.max_error_bits
                  << ", decision_margin_cutoff=" << config.min_decision_margin << '\n';
        std::cerr << "Output orientation: " << orientationName(output_orientation) << '\n';
        std::cerr << "Preprocessing backend: " << preprocessor.backend() << '\n';

        cv::VideoWriter writer;
        if (cli.record_path.has_value()) {
            const double record_fps =
                actual_fps > 1.0 ? actual_fps : static_cast<double>(cli.fps.value_or(30));
            applyOutputOrientation(frame, output_frame, output_orientation);
            if (!writer.open(*cli.record_path,
                             cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                             record_fps,
                             cv::Size(output_frame.cols, output_frame.rows))) {
                std::cerr << "Failed to open output video: " << *cli.record_path << '\n';
                return 1;
            }
        }
        AprilTagDetector detector(config);

        PublisherOptions publisher_options;
        publisher_options.camera_name = cli.camera_name;
        publisher_options.team_number = cli.team_number;
        publisher_options.server = cli.nt_server;
        publisher_options.use_stdout_json = cli.stdout_json;
        publisher_options.field_layout_loaded = field_layout.has_value();
        publisher_options.field_layout_tag_count =
            field_layout.has_value() ? static_cast<int>(field_layout->tags_by_id.size()) : 0;
        if (field_layout.has_value()) {
            publisher_options.field_length_m = field_layout->field_length_m;
            publisher_options.field_width_m = field_layout->field_width_m;
        }
        Publisher publisher(publisher_options);

#if HAVE_NTCORE
        applyOutputOrientation(frame, output_frame, output_orientation);
        const int stream_width = cli.stream_width.value_or(output_frame.cols);
        const int stream_height = cli.stream_height.value_or(output_frame.rows);
        cs::CvSource output_stream = frc::CameraServer::PutVideo(cli.camera_name, stream_width, stream_height);
        output_stream.SetResolution(stream_width, stream_height);
        if (actual_fps > 1.0) {
            output_stream.SetFPS(static_cast<int>(std::lround(actual_fps)));
        } else if (cli.fps.has_value()) {
            output_stream.SetFPS(*cli.fps);
        }
#else
        if (cli.nt_server.has_value() || cli.team_number.has_value()) {
            std::cerr << "NetworkTables support is disabled in this build.\n";
        }
#endif

        std::int64_t frame_id = 0;
        double displayed_fps = 0.0;
        int frames_since_tick = 0;
        auto tick_start = SteadyClock::now();
        cv::Mat gray;

        while (!g_stop.load()) {
            const auto frame_start = SteadyClock::now();
            preprocessor.toGray(frame, gray);

            const auto detections = detector.detect(gray);

            std::vector<DetectionResult> out;
            out.reserve(detections.size());

            for (apriltag_detection_t* det : detections) {
                try {
                    PoseResult pose = estimatePose(
                        det,
                        camera_matrix,
                        dist_coeffs,
                        config.tag_size_meters,
                        config.pose_refine_iterations);
                    out.push_back({det, pose});
                    drawDetection(frame, det, pose, camera_matrix, dist_coeffs);
                } catch (const std::exception& e) {
                    std::cerr << "Pose estimation failed for tag id=" << det->id << ": " << e.what() << '\n';
                }
            }

            std::optional<FieldPoseResult> field_pose;
            if (field_layout.has_value()) {
                try {
                    field_pose =
                        estimateFieldPose(
                            out,
                            *field_layout,
                            camera_matrix,
                            dist_coeffs,
                            config.tag_size_meters,
                            config.pose_refine_iterations);
                } catch (const std::exception& e) {
                    std::cerr << "Field pose estimation failed: " << e.what() << '\n';
                }
            }

            frames_since_tick++;
            const auto now = SteadyClock::now();
            const double elapsed = std::chrono::duration<double>(now - tick_start).count();
            if (elapsed > 0.0) {
                displayed_fps = static_cast<double>(frames_since_tick) / elapsed;
            }
            if (elapsed >= 1.0) {
                std::cerr << "FPS: " << displayed_fps << '\n';
                tick_start = now;
                frames_since_tick = 0;
            }

            drawFps(frame, displayed_fps);
            drawFieldPoseSummary(frame, field_pose);

            const double latency_ms =
                std::chrono::duration<double, std::milli>(SteadyClock::now() - frame_start).count();
            publisher.publish(true, frame_id, frame_timestamp_us, latency_ms, displayed_fps, out, field_pose);
            applyOutputOrientation(frame, output_frame, output_orientation);
#if HAVE_NTCORE
            if (cli.stream_width.has_value() && cli.stream_height.has_value()) {
                cv::resize(
                    output_frame,
                    stream_frame,
                    cv::Size(*cli.stream_width, *cli.stream_height),
                    0.0,
                    0.0,
                    cv::INTER_LINEAR);
                output_stream.PutFrame(stream_frame);
            } else {
                output_stream.PutFrame(output_frame);
            }
#endif

            if (!cli.no_display) {
                cv::imshow("apriltag_vision", output_frame);
                const int key = cv::waitKey(1);
                if (key == 'q' || key == 'Q') {
                    break;
                }
            }

            if (writer.isOpened()) {
                writer.write(output_frame);
            }

            ++frame_id;
            if (!readFrame(cap, frame, frame_timestamp_us)) {
                std::cerr << "Camera frame read failed.\n";
                break;
            }
        }

        publisher.publish(false, frame_id, nowUnixMicros(), 0.0, displayed_fps, {}, std::nullopt);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        std::cerr
            << "Usage: apriltag_vision --calibration <path> [--camera <int>] [--tag-size <meters>] "
               "[--camera-name <name>] [--team <number> | --nt-server <host>] [--width <px>] "
               "[--height <px>] [--fps <fps>] [--family <tag36h11>] [--threads <count>] "
               "[--quad-decimate <value>] [--blur <sigma>] [--refine-edges <on|off>] "
               "[--pose-iterations <0-100>] [--max-error-bits <count>] "
               "[--decision-margin-cutoff <value>] [--auto-exposure <on|off>] "
               "[--exposure <value>] [--brightness <value>] [--auto-white-balance <on|off>] "
               "[--white-balance <kelvin>] [--orientation <normal|cw90|180|ccw90>] "
               "[--stream-width <px> --stream-height <px>] [--field-layout <layout.json>] "
               "[--no-display] [--stdout-json] [--record <output.mp4>]\n";
        return 1;
    }
}
