#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include "calibration.hpp"
#include "config.hpp"
#include "detector.hpp"
#include "pose.hpp"
#include "publisher.hpp"
#include "visualize.hpp"

#if HAVE_NTCORE
#include <cameraserver/CameraServer.h>
#include <cscore_cv.h>
#endif

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
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
    std::optional<double> decision_margin;
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
        } else if (arg == "--decision-margin") {
            opts.decision_margin =
                parseNonNegativeDouble(requireValue("--decision-margin"), "--decision-margin");
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
        if (cli.threads.has_value()) {
            config.nthreads = *cli.threads;
        }
        if (cli.quad_decimate.has_value()) {
            config.quad_decimate = *cli.quad_decimate;
        }
        if (cli.decision_margin.has_value()) {
            config.min_decision_margin = *cli.decision_margin;
        }

        cv::Mat camera_matrix, dist_coeffs;
        try {
            loadCalibration(cli.calibration_path, camera_matrix, dist_coeffs);
        } catch (const std::exception& e) {
            std::cerr << "Calibration error: " << e.what() << '\n';
            return 1;
        }

        cv::VideoCapture cap(config.camera_index);
        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera index " << config.camera_index << '\n';
            return 1;
        }

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

        cv::Mat frame;
        std::int64_t frame_timestamp_us = 0;
        if (!readFrame(cap, frame, frame_timestamp_us)) {
            std::cerr << "Camera frame read failed.\n";
            return 1;
        }

        const double actual_fps = cap.get(cv::CAP_PROP_FPS);
        std::cerr << "Camera mode: " << frame.cols << 'x' << frame.rows;
        if (actual_fps > 1.0) {
            std::cerr << " @ " << actual_fps << " FPS";
        }
        std::cerr << '\n';

        cv::VideoWriter writer;
        if (cli.record_path.has_value()) {
            const double record_fps =
                actual_fps > 1.0 ? actual_fps : static_cast<double>(cli.fps.value_or(30));
            if (!writer.open(*cli.record_path,
                             cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                             record_fps,
                             cv::Size(frame.cols, frame.rows))) {
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
        Publisher publisher(publisher_options);

#if HAVE_NTCORE
        cs::CvSource output_stream = frc::CameraServer::PutVideo(cli.camera_name, frame.cols, frame.rows);
        output_stream.SetResolution(frame.cols, frame.rows);
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
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            const auto detections = detector.detect(gray);

            std::vector<std::pair<apriltag_detection_t*, PoseResult>> out;
            out.reserve(detections.size());

            for (apriltag_detection_t* det : detections) {
                try {
                    PoseResult pose = estimatePose(det, camera_matrix, dist_coeffs, config.tag_size_meters);
                    out.emplace_back(det, pose);
                    drawDetection(frame, det, pose, camera_matrix, dist_coeffs);
                } catch (const std::exception& e) {
                    std::cerr << "Pose estimation failed for tag id=" << det->id << ": " << e.what() << '\n';
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

            const double latency_ms =
                std::chrono::duration<double, std::milli>(SteadyClock::now() - frame_start).count();
            publisher.publish(true, frame_id, frame_timestamp_us, latency_ms, displayed_fps, out);
#if HAVE_NTCORE
            output_stream.PutFrame(frame);
#endif

            if (!cli.no_display) {
                cv::imshow("apriltag_vision", frame);
                const int key = cv::waitKey(1);
                if (key == 'q' || key == 'Q') {
                    break;
                }
            }

            if (writer.isOpened()) {
                writer.write(frame);
            }

            ++frame_id;
            if (!readFrame(cap, frame, frame_timestamp_us)) {
                std::cerr << "Camera frame read failed.\n";
                break;
            }
        }

        publisher.publish(false, frame_id, nowUnixMicros(), 0.0, displayed_fps, {});
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        std::cerr
            << "Usage: apriltag_vision --calibration <path> [--camera <int>] [--tag-size <meters>] "
               "[--camera-name <name>] [--team <number> | --nt-server <host>] [--width <px>] "
               "[--height <px>] [--fps <fps>] [--threads <count>] [--quad-decimate <value>] "
               "[--decision-margin <value>] [--no-display] [--stdout-json] [--record <output.mp4>]\n";
        return 1;
    }
}
