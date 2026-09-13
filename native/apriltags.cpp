// AprilTag 3 + OpenCV native hot path. The GIL is released for all frame work.
// Corner convention: apriltag_detection_t::p follows homography corners
// (-1,+1), (+1,+1), (+1,-1), (-1,-1), also OpenCV IPPE_SQUARE order.
// References: https://github.com/AprilRobotics/apriltag
// https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#ifdef CUSTOM_VISION_HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
#include <cuAprilTags.h>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudawarping.hpp>
#endif
extern "C" {
#include <apriltag.h>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
zarray_t* vision_apriltag_verify_quads(apriltag_detector_t*, image_u8_t*, const float*, int);
#endif
}
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace py = pybind11;
using Clock = std::chrono::steady_clock;
using Corners = std::array<cv::Point2d, 4>;
using Vec3 = std::array<double, 3>;
static double milliseconds(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

static double numeric(const py::dict& config, const char* key, double fallback, bool zero = false) {
    py::object value = config.contains(key) ? py::reinterpret_borrow<py::object>(config[key]) : py::float_(fallback);
    if (py::isinstance<py::bool_>(value)) throw py::value_error(std::string(key) + " must be numeric");
    double result;
    try { result = py::cast<double>(value); }
    catch (const py::cast_error&) { throw py::value_error(std::string(key) + " must be numeric"); }
    if (!std::isfinite(result) || result < 0 || (!zero && result == 0))
        throw py::value_error(std::string(key) + " must be finite and " + (zero ? "nonnegative" : "positive"));
    return result;
}

static int integer(const py::dict& config, const char* key, int fallback, int low, int high) {
    if (!config.contains(key)) return fallback;
    const auto value = config[key];
    if (py::isinstance<py::bool_>(value) || !py::isinstance<py::int_>(value))
        throw py::value_error(std::string(key) + " must be an integer");
    const auto result = py::cast<long long>(value);
    if (result < low || result > high) throw py::value_error(std::string(key) + " is outside supported range");
    return static_cast<int>(result);
}

struct Family {
    apriltag_family_t* ptr = nullptr;
    void (*destroy)(apriltag_family_t*) = nullptr;
    ~Family() { if (ptr) destroy(ptr); }
    void create(const std::string& name) {
#define TAG_FAMILY(n) if (name == #n) { ptr = n##_create(); destroy = n##_destroy; return; }
        TAG_FAMILY(tag16h5) TAG_FAMILY(tag25h9) TAG_FAMILY(tag36h11)
        TAG_FAMILY(tagCircle21h7) TAG_FAMILY(tagCircle49h12) TAG_FAMILY(tagCustom48h12)
        TAG_FAMILY(tagStandard41h12) TAG_FAMILY(tagStandard52h13)
#undef TAG_FAMILY
        throw py::value_error("Unsupported AprilTag family: " + name);
    }
};
struct DetectorDeleter { void operator()(apriltag_detector_t* ptr) const { if (ptr) apriltag_detector_destroy(ptr); } };
struct DetectionsDeleter { void operator()(zarray_t* ptr) const { if (ptr) apriltag_detections_destroy(ptr); } };
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
struct GpuDetectorHandle {
    cuAprilTagsHandle value = nullptr;
    ~GpuDetectorHandle() { if (value) cuAprilTagsDestroy(value); }
};
#endif

struct Candidate { cv::Vec3d rvec{}, tvec{}; double error = std::numeric_limits<double>::infinity(); };
struct Pose {
    bool valid = false;
    std::string reason = "pnp_failed";
    Candidate best, alternate;
    bool has_alternate = false;
    // Ratio from the two original IPPE hypotheses, before LM refinement can
    // converge both to one solution. 1 means indistinguishable; 0 means distinct.
    double ambiguity = -1;
};
struct Detection {
    int id, hamming;
    double margin;
    std::array<double, 2> center;
    Corners corners;
    Pose pose;
};
struct Timings { double preprocess = 0, detect = 0, pose = 0, total = 0; };

static Vec3 as_array(const cv::Vec3d& value) { return {value[0], value[1], value[2]}; }
static py::dict pose_dictionary(const Pose& pose) {
    py::dict result;
    result["pose_valid"] = pose.valid;
    if (!pose.valid) {
        result["pose_invalid_reason"] = pose.reason;
        if (std::isfinite(pose.best.error)) result["reprojection_error_px"] = pose.best.error;
        return result;
    }
    result["tvec_m"] = as_array(pose.best.tvec);
    result["rvec_rad"] = as_array(pose.best.rvec);
    result["distance_m"] = cv::norm(pose.best.tvec);
    result["reprojection_error_px"] = pose.best.error;
    result["pose_ambiguity"] = pose.ambiguity;
    if (pose.has_alternate) {
        result["alternate_tvec_m"] = as_array(pose.alternate.tvec);
        result["alternate_rvec_rad"] = as_array(pose.alternate.rvec);
        result["alternate_reprojection_error_px"] = pose.alternate.error;
    }
    return result;
}

class Detector {
public:
    Detector(const py::dict& config, py::object calibration) {
        tag_size_ = numeric(config, "tag_size_m", 0.1651);
        margin_ = numeric(config, "min_decision_margin", 30.0, true);
        max_error_ = numeric(config, "max_reprojection_error_px", 3.0);
        const double decimate = numeric(config, "quad_decimate", 2.0);
        if (decimate < 1 || decimate > 8) throw py::value_error("quad_decimate must be between 1 and 8");
        quad_decimate_ = decimate;
        const double sigma = numeric(config, "quad_sigma", 0.0, true);
        const double sharpen = numeric(config, "decode_sharpening", 0.25, true);
        const int threads = integer(config, "threads", 2, 1, 64);
        hamming_ = integer(config, "max_hamming", 0, 0, 2);
        mode_ = config.contains("mode") ? py::cast<std::string>(config["mode"]) : "3d";
        if (mode_ != "2d" && mode_ != "3d") throw py::value_error("mode must be 2d or 3d");
        if (config.contains("skip_single_when_multi")) {
            if (!py::isinstance<py::bool_>(config["skip_single_when_multi"]))
                throw py::value_error("skip_single_when_multi must be boolean");
            skip_single_when_multi_ = py::cast<bool>(config["skip_single_when_multi"]);
        }
        if (config.contains("known_tag_ids")) {
            if (!py::isinstance<py::list>(config["known_tag_ids"]))
                throw py::value_error("known_tag_ids must be a list of nonnegative integers");
            for (const auto value : py::cast<py::list>(config["known_tag_ids"])) {
                if (!py::isinstance<py::int_>(value) || py::isinstance<py::bool_>(value))
                    throw py::value_error("known_tag_ids must be a list of nonnegative integers");
                const auto id = py::cast<long long>(value);
                if (id < 0 || id > std::numeric_limits<int>::max())
                    throw py::value_error("known_tag_ids contains an unsupported ID");
                known_ids_.insert(static_cast<int>(id));
            }
        }
        const std::string preprocess = config.contains("preprocess") ? py::cast<std::string>(config["preprocess"]) : "cpu";
        if (preprocess != "cpu" && preprocess != "cuda") throw py::value_error("preprocess must be cpu or cuda");
        const std::string device = config.contains("detector_device") ? py::cast<std::string>(config["detector_device"]) : "cpu";
        if (device != "cpu" && device != "cuda") throw py::value_error("detector_device must be cpu or cuda");
        cuda_detect_ = device == "cuda";
#ifndef CUSTOM_VISION_HAVE_CUAPRILTAGS
        if (cuda_detect_) throw py::value_error("CUDA AprilTag detector is not compiled; build with -DCUSTOM_VISION_CUDA_APRILTAGS=ON on JetPack 6");
#else
        gpu_max_tags_ = integer(config, "cuda_max_tags", 32, 1, 256);
        gpu_tags_.resize(gpu_max_tags_);
#endif
        cuda_ = preprocess == "cuda";
        if (cuda_ || cuda_detect_) {
#ifdef CUSTOM_VISION_HAVE_CUDA
            if (cv::cuda::getCudaEnabledDeviceCount() < 1) throw py::value_error("CUDA preprocessing requested but no CUDA device is available");
            stream_ = std::make_unique<cv::cuda::Stream>();
#else
            throw py::value_error("CUDA preprocessing was not compiled; build with OpenCV cudaimgproc support");
#endif
        }
        if (!calibration.is_none()) {
            // Use the same strict pinhole validation at both Python entry points;
            // it runs once at setup, never in the per-frame path.
            py::dict checked = py::module_::import("custom_vision.calibration").attr("validate_calibration")(calibration);
            width_ = py::cast<int>(checked["width"]);
            height_ = py::cast<int>(checked["height"]);
            const auto matrix = py::cast<std::vector<std::vector<double>>>(checked["camera_matrix"]);
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int y = 0; y < 3; ++y) for (int x = 0; x < 3; ++x) camera_matrix_.at<double>(y, x) = matrix[y][x];
            const auto coefficients = py::cast<std::vector<double>>(checked["dist_coeffs"]);
            distortion_ = cv::Mat(coefficients, true);
        }
        const double half = tag_size_ / 2;
        object_points_ = {{-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};
        const std::string family = config.contains("tag_family") ? py::cast<std::string>(config["tag_family"]) : "tag36h11";
        if (cuda_detect_ && family != "tag36h11") throw py::value_error("CUDA AprilTag supports tag36h11 only");
        family_.create(family);
        detector_.reset(apriltag_detector_create());
        if (!family_.ptr || !detector_) throw std::runtime_error("Could not allocate AprilTag detector");
        detector_->nthreads = threads;
        detector_->quad_decimate = static_cast<float>(decimate);
        detector_->quad_sigma = static_cast<float>(sigma);
        detector_->decode_sharpening = sharpen;
        // Explicit quality/speed tradeoff: ignore low-contrast image tiles.
        // Preserve upstream's default; tune against camera noise and distant
        // tag recall before increasing it for a deployment.
        detector_->qtp.min_white_black_diff = integer(config, "min_white_black_diff", 5, 1, 255);
        if (config.contains("refine_edges") && !py::isinstance<py::bool_>(config["refine_edges"]))
            throw py::value_error("refine_edges must be boolean");
        detector_->refine_edges = config.contains("refine_edges") ? py::cast<bool>(config["refine_edges"]) : true;
        if (cuda_detect_ && !detector_->refine_edges)
            throw py::value_error("CUDA AprilTag requires refine_edges=true to recover raw-image corners after GPU sampling");
        detector_->debug = false;
        apriltag_detector_add_family_bits(detector_.get(), family_.ptr, hamming_);
    }

    py::list process(py::array frame) {
        const auto input = frame.request();
        const bool gray = input.ndim == 2;
        if (!frame.dtype().is(py::dtype::of<uint8_t>()) || (input.ndim != 2 && input.ndim != 3)
            || (input.ndim == 3 && input.shape[2] != 3)
            || input.shape[0] < 8 || input.shape[1] < 8
            || input.shape[0] > std::numeric_limits<int>::max() || input.shape[1] > std::numeric_limits<int>::max())
            throw py::value_error("AprilTag input must be a uint8 gray or BGR image, at least 8x8");
        const int channels = gray ? 1 : 3;
        if (input.strides[0] < input.shape[1] * channels || input.strides[1] != channels
            || (!gray && input.strides[2] != 1) || input.strides[0] > std::numeric_limits<int>::max())
            throw py::value_error("AprilTag input requires contiguous pixels and a positive row stride; use np.ascontiguousarray after slicing/rotation");
        cv::Mat view(static_cast<int>(input.shape[0]), static_cast<int>(input.shape[1]),
                     gray ? CV_8UC1 : CV_8UC3, input.ptr, static_cast<size_t>(input.strides[0]));
        std::vector<Detection> detections;
        {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(mutex_);
            const auto start = Clock::now();
            cv::Mat gray_frame;
            if (gray) gray_frame = view; // zero-copy mono path, independent of preprocess setting
            else if (!cuda_) {
                cv::cvtColor(view, gray_buffer_, cv::COLOR_BGR2GRAY);
                gray_frame = gray_buffer_;
            } else {
#ifdef CUSTOM_VISION_HAVE_CUDA
                // Reuse allocations across frames. Stream operations are ordered;
                // wait before the CPU detector touches the grayscale output.
                gpu_bgr_.upload(view, *stream_);
                cv::cuda::cvtColor(gpu_bgr_, gpu_gray_, cv::COLOR_BGR2GRAY, 0, *stream_);
                gpu_gray_.download(gray_buffer_, *stream_);
                stream_->waitForCompletion();
                gray_frame = gray_buffer_;
#endif
            }
            const auto preprocessed = Clock::now();
            image_u8_t image{gray_frame.cols, gray_frame.rows, static_cast<int>(gray_frame.step), gray_frame.data};
            std::unique_ptr<zarray_t, DetectionsDeleter> native;
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
            if (cuda_detect_) native.reset(detect_gpu(view, image));
            else
#endif
                native.reset(apriltag_detector_detect(detector_.get(), &image));
            if (!native) throw std::runtime_error("AprilTag detector failed");
            const auto detected = Clock::now();
            std::string reason;
            if (mode_ == "2d") reason = "mode_2d";
            else if (camera_matrix_.empty()) reason = "no_calibration";
            else if (width_ != view.cols || height_ != view.rows) reason = "calibration_resolution_mismatch";
            const int count = zarray_size(native.get());
            detections.reserve(count);
            for (int i = 0; i < count; ++i) {
                apriltag_detection_t* tag = nullptr;
                zarray_get(native.get(), i, &tag);
                if (!std::isfinite(tag->decision_margin) || tag->decision_margin < margin_ || tag->hamming > hamming_) continue;
                Detection detection;
                detection.id = tag->id; detection.hamming = tag->hamming; detection.margin = tag->decision_margin;
                detection.center = {tag->c[0], tag->c[1]};
                bool finite = std::isfinite(tag->c[0]) && std::isfinite(tag->c[1]);
                for (int j = 0; j < 4; ++j) {
                    detection.corners[j] = {tag->p[j][0], tag->p[j][1]};
                    finite = finite && std::isfinite(tag->p[j][0]) && std::isfinite(tag->p[j][1]);
                }
                if (!finite) continue;
                detections.push_back(std::move(detection));
            }
            // Joint field PnP uses original corners. Avoid solving every tag
            // first when the localization stage can derive individual poses
            // from its joint result. Count unique quality-valid mapped IDs only.
            if (reason.empty() && skip_single_when_multi_) {
                int first_known = -1;
                for (const auto& detection : detections) {
                    if (!known_ids_.count(detection.id)) continue;
                    if (first_known >= 0 && first_known != detection.id) {
                        reason = "deferred_multitag";
                        break;
                    }
                    first_known = detection.id;
                }
            }
            for (auto& detection : detections) {
                if (reason.empty()) detection.pose = estimate(detection.corners);
                else detection.pose.reason = reason;
            }
            const auto posed = Clock::now();
            timings_ = {milliseconds(start, preprocessed), milliseconds(preprocessed, detected),
                        milliseconds(detected, posed), milliseconds(start, posed)};
        }
        py::list output;
        for (const auto& detection : detections) {
            py::dict value = pose_dictionary(detection.pose);
            value["id"] = detection.id; value["hamming"] = detection.hamming;
            value["decision_margin"] = detection.margin; value["center"] = detection.center;
            std::array<std::array<double, 2>, 4> corners;
            for (int i = 0; i < 4; ++i) corners[i] = {detection.corners[i].x, detection.corners[i].y};
            value["corners"] = corners;
            output.append(std::move(value));
        }
        return output;
    }

    py::dict estimate_pose(py::array_t<double, py::array::c_style | py::array::forcecast> image_points) {
        const auto input = image_points.request();
        if (input.ndim != 2 || input.shape[0] != 4 || input.shape[1] != 2)
            throw py::value_error("Pose corners must have shape (4, 2)");
        const auto* data = static_cast<double*>(input.ptr);
        Corners corners;
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(data[2 * i]) || !std::isfinite(data[2 * i + 1])) throw py::value_error("Pose corners must be finite");
            corners[i] = {data[2 * i], data[2 * i + 1]};
        }
        Pose pose;
        if (mode_ == "2d") pose.reason = "mode_2d";
        else if (camera_matrix_.empty()) pose.reason = "no_calibration";
        else {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(mutex_);
            pose = estimate(corners);
        }
        return pose_dictionary(pose);
    }

    py::dict last_timings() {
        Timings timings;
        { std::lock_guard<std::mutex> guard(mutex_); timings = timings_; }
        py::dict result;
        result["preprocess_ms"] = timings.preprocess; result["detect_ms"] = timings.detect;
        result["pose_ms"] = timings.pose; result["total_ms"] = timings.total;
        return result;
    }

    py::dict last_profile() {
        // AprilTag already records these stamps. Read them only on demand;
        // normal frame processing performs no additional timing/allocation work.
        std::lock_guard<std::mutex> guard(mutex_);
        py::dict phases;
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
        if (cuda_detect_) {
            phases["gpu_upload_expand_rectify"] = gpu_prepare_ms_;
            phases["gpu_detect_decode"] = gpu_detect_ms_;
            phases["cpu_verify_refine"] = gpu_verify_ms_;
            py::dict result;
            result["phases_ms"] = phases;
            result["gpu_tag_count"] = gpu_count_;
            result["capacity_reached"] = gpu_count_ >= static_cast<uint32_t>(gpu_max_tags_);
            return result;
        }
#endif
        auto last = detector_->tp->utime;
        for (int i = 0; i < zarray_size(detector_->tp->stamps); ++i) {
            timeprofile_entry entry;
            zarray_get(detector_->tp->stamps, i, &entry);
            phases[entry.name] = std::max<int64_t>(0, entry.utime - last) / 1000.0;
            last = entry.utime;
        }
        py::dict result;
        result["phases_ms"] = phases;
        result["quad_count"] = detector_->nquads;
        return result;
    }

private:
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
    zarray_t* detect_gpu(const cv::Mat& input, image_u8_t& gray) {
        if (input.cols > 65535 || input.rows > 65535)
            throw std::invalid_argument("CUDA AprilTag image dimensions must fit uint16");
        const auto start = Clock::now();
        const bool rectified = !camera_matrix_.empty() && width_ == input.cols && height_ == input.rows
                               && cv::norm(distortion_) > 0;
        if (gpu_width_ != input.cols || gpu_height_ != input.rows) {
            gpu_detection_width_ = static_cast<int>(std::round(input.cols / quad_decimate_));
            gpu_detection_height_ = static_cast<int>(std::round(input.rows / quad_decimate_));
            if (gpu_detection_width_ < 8 || gpu_detection_height_ < 8)
                throw std::invalid_argument("CUDA AprilTag decimated image must be at least 8x8");
            if (gpu_detector_.value) {
                cuAprilTagsDestroy(gpu_detector_.value);
                gpu_detector_.value = nullptr;
            }
            const int error = nvCreateAprilTagsDetector(&gpu_detector_.value, gpu_detection_width_, gpu_detection_height_, 4,
                                                        NVAT_TAG36H11, nullptr, static_cast<float>(tag_size_));
            if (error) throw std::runtime_error("cuAprilTags create failed, code " + std::to_string(error));
            gpu_width_ = input.cols; gpu_height_ = input.rows;
            if (rectified) {
                cv::Mat map_x, map_y;
                cv::initUndistortRectifyMap(camera_matrix_, distortion_, cv::Mat(), camera_matrix_,
                                           input.size(), CV_32FC1, map_x, map_y);
                gpu_map_x_.upload(map_x); gpu_map_y_.upload(map_y);
            }
        }
        if (input.channels() == 1) {
            gpu_gray_.upload(input, *stream_);
            cv::cuda::cvtColor(gpu_gray_, gpu_bgr_, cv::COLOR_GRAY2BGR, 0, *stream_);
        } else if (!cuda_) gpu_bgr_.upload(input, *stream_);
        cv::cuda::GpuMat detection_input = gpu_bgr_;
        if (rectified) {
            cv::cuda::remap(gpu_bgr_, gpu_rectified_, gpu_map_x_, gpu_map_y_, cv::INTER_LINEAR,
                           cv::BORDER_CONSTANT, cv::Scalar(), *stream_);
            detection_input = gpu_rectified_;
        }
        if (gpu_detection_width_ != input.cols || gpu_detection_height_ != input.rows) {
            cv::cuda::resize(detection_input, gpu_downsampled_, cv::Size(gpu_detection_width_, gpu_detection_height_),
                             0, 0, cv::INTER_LINEAR, *stream_);
            detection_input = gpu_downsampled_;
        }
        // Finish transfers/rectification before entering a third-party library.
        stream_->waitForCompletion();
        const auto prepared = Clock::now();
        cuAprilTagsImageInput_t view{reinterpret_cast<uchar3*>(detection_input.data), detection_input.step,
                                     static_cast<uint16_t>(detection_input.cols), static_cast<uint16_t>(detection_input.rows)};
        gpu_count_ = 0;
        const int error = cuAprilTagsDetect(gpu_detector_.value, &view, gpu_tags_.data(), &gpu_count_,
                                           gpu_max_tags_, cv::cuda::StreamAccessor::getStream(*stream_));
        stream_->waitForCompletion();
        if (error) throw std::runtime_error("cuAprilTags detection failed, code " + std::to_string(error));
        if (gpu_count_ > static_cast<uint32_t>(gpu_max_tags_))
            throw std::runtime_error("cuAprilTags returned an invalid detection count");
        const auto detected = Clock::now();
        gpu_corners_.clear();
        gpu_corners_.reserve(gpu_count_ * 8);
        for (uint32_t index = 0; index < gpu_count_; ++index) {
            const auto& tag = gpu_tags_[index];
            if (tag.hamming_error > hamming_) continue;
            std::vector<cv::Point2d> corners;
            std::vector<cv::Point3d> normalized;
            bool finite = true;
            for (int i = 0; i < 4; ++i) {
                // Undo OpenCV resize's pixel-center transform before lens
                // distortion. Full-resolution CPU refinement then removes
                // the GPU detector's finite-pixel boundary bias.
                const double x = (tag.corners[i].x + 0.5) * input.cols / static_cast<double>(gpu_detection_width_) - 0.5;
                const double y = (tag.corners[i].y + 0.5) * input.rows / static_cast<double>(gpu_detection_height_) - 0.5;
                finite = finite && std::isfinite(x) && std::isfinite(y);
                corners.emplace_back(x, y);
                if (rectified) normalized.emplace_back((x-camera_matrix_.at<double>(0,2))/camera_matrix_.at<double>(0,0),
                                                       (y-camera_matrix_.at<double>(1,2))/camera_matrix_.at<double>(1,1), 1);
            }
            if (!finite) continue;
            if (rectified) cv::projectPoints(normalized, cv::Vec3d(), cv::Vec3d(), camera_matrix_, distortion_, corners);
            // Decoder determines starting corner from the code bits. Only fix
            // polygon winding here; do not rely on undocumented GPU corner order.
            double area = 0;
            for (int i = 0; i < 4; ++i) area += corners[i].x * corners[(i+1)%4].y - corners[(i+1)%4].x * corners[i].y;
            if (!std::isfinite(area) || std::abs(area) < 1) continue;
            if (area < 0) std::reverse(corners.begin(), corners.end());
            for (const auto& point : corners) { gpu_corners_.push_back(point.x); gpu_corners_.push_back(point.y); }
        }
        auto* result = vision_apriltag_verify_quads(detector_.get(), &gray, gpu_corners_.data(), gpu_corners_.size()/8);
        const auto verified = Clock::now();
        gpu_prepare_ms_ = milliseconds(start, prepared); gpu_detect_ms_ = milliseconds(prepared, detected);
        gpu_verify_ms_ = milliseconds(detected, verified);
        return result;
    }
#endif
    double reprojection(Candidate& candidate, const Corners& image_points) const {
        for (int i = 0; i < 3; ++i) if (!std::isfinite(candidate.rvec[i]) || !std::isfinite(candidate.tvec[i])) return INFINITY;
        cv::Matx33d rotation;
        cv::Rodrigues(candidate.rvec, rotation);
        for (const auto& point : object_points_) {
            const auto transformed = rotation * cv::Vec3d(point.x, point.y, point.z) + candidate.tvec;
            if (transformed[2] <= 0) return INFINITY;
        }
        std::vector<cv::Point2d> projected;
        cv::projectPoints(object_points_, candidate.rvec, candidate.tvec, camera_matrix_, distortion_, projected);
        double sum = 0;
        for (int i = 0; i < 4; ++i) { const auto diff = projected[i] - image_points[i]; sum += diff.dot(diff); }
        return std::sqrt(sum / 4);
    }

    Pose estimate(const Corners& corners) const {
        Pose result;
        try {
            std::vector<cv::Vec3d> rvecs, tvecs;
            const std::vector<cv::Point2d> pixels(corners.begin(), corners.end());
            // IPPE's Rodrigues conversion can become singular for an exactly
            // frontal marker whose decoded frame is rotated pi in the image.
            // Choose a cyclic solver frame and map its rotation back afterward.
            // This never changes the public decoded corner order or tag frame.
            int shift = 0;
            for (int i = 1; i < 4; ++i)
                if (corners[i].y - corners[i].x > corners[shift].y - corners[shift].x) shift = i;
            std::vector<cv::Point2d> solver_pixels;
            solver_pixels.reserve(4);
            for (int i = 0; i < 4; ++i) solver_pixels.push_back(corners[(i + shift) % 4]);
            const double angle = shift * CV_PI / 2;
            const cv::Matx33d restore(std::cos(angle), -std::sin(angle), 0,
                                     std::sin(angle), std::cos(angle), 0, 0, 0, 1);
            if (!cv::solvePnPGeneric(object_points_, solver_pixels, camera_matrix_, distortion_, rvecs, tvecs,
                                     false, cv::SOLVEPNP_IPPE_SQUARE)) return result;
            std::vector<Candidate> candidates;
            for (size_t i = 0; i < rvecs.size(); ++i) {
                Candidate candidate{rvecs[i], tvecs[i]};
                cv::Matx33d solver_rotation;
                cv::Rodrigues(candidate.rvec, solver_rotation);
                cv::Rodrigues(solver_rotation * restore, candidate.rvec);
                candidate.error = reprojection(candidate, corners);
                if (std::isfinite(candidate.error)) candidates.push_back(candidate);
            }
            if (candidates.empty()) return result;
            std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.error < b.error; });
            result.best = candidates[0];
            if (candidates.size() > 1) {
                result.has_alternate = true;
                result.alternate = candidates[1];
                result.ambiguity = candidates[1].error < 1e-9 ? 1.0 : std::clamp(candidates[0].error / candidates[1].error, 0.0, 1.0);
            }
            // Refine only the best IPPE hypothesis; preserve the alternate and
            // original ambiguity rather than collapsing two starts to one pose.
            Candidate refined = result.best;
            if (cv::solvePnP(object_points_, pixels, camera_matrix_, distortion_, refined.rvec, refined.tvec,
                             true, cv::SOLVEPNP_ITERATIVE)) {
                refined.error = reprojection(refined, corners);
                if (std::isfinite(refined.error) && refined.error < result.best.error) result.best = refined;
            }
            if (result.best.error > max_error_) { result.reason = "reprojection_error"; return result; }
            result.valid = true;
            result.reason.clear();
        } catch (const cv::Exception&) { return result; }
        return result;
    }

    // Detector must be destroyed before its family (reverse member order).
    Family family_;
    std::unique_ptr<apriltag_detector_t, DetectorDeleter> detector_;
    std::mutex mutex_;
    double tag_size_, margin_, max_error_, quad_decimate_;
    int hamming_, width_ = 0, height_ = 0;
    bool cuda_ = false;
    bool cuda_detect_ = false;
    bool skip_single_when_multi_ = false;
    std::unordered_set<int> known_ids_;
    std::string mode_;
    cv::Mat camera_matrix_, distortion_, gray_buffer_;
    std::vector<cv::Point3d> object_points_;
    Timings timings_;
#ifdef CUSTOM_VISION_HAVE_CUDA
    cv::cuda::GpuMat gpu_bgr_, gpu_gray_;
    std::unique_ptr<cv::cuda::Stream> stream_;
#endif
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
    GpuDetectorHandle gpu_detector_;
    int gpu_width_ = 0, gpu_height_ = 0, gpu_max_tags_ = 32;
    int gpu_detection_width_ = 0, gpu_detection_height_ = 0;
    uint32_t gpu_count_ = 0;
    std::vector<cuAprilTagsID_t> gpu_tags_;
    std::vector<float> gpu_corners_;
    cv::cuda::GpuMat gpu_rectified_, gpu_downsampled_, gpu_map_x_, gpu_map_y_;
    double gpu_prepare_ms_ = 0, gpu_detect_ms_ = 0, gpu_verify_ms_ = 0;
#endif
};

static py::dict capabilities() {
    py::dict result;
    result["apriltag_device"] = "cpu";
    result["pose_device"] = "cpu";
    result["apriltag_version"] = CUSTOM_VISION_APRILTAG_VERSION;
    result["opencv_version"] = CV_VERSION;
    result["opencv_threads"] = cv::getNumThreads();
    result["gil_released"] = true;
    result["gray_zero_copy"] = true;
#ifdef CUSTOM_VISION_HAVE_CUAPRILTAGS
    result["cuda_apriltag_compiled"] = true;
    result["cuda_apriltag_backend"] = "cuAprilTags Isaac ROS 3.2 / JetPack 6.1";
    result["cuda_apriltag_license"] = "NVIDIA ISAAC ROS SOFTWARE LICENSE";
    result["cuda_apriltag_cpu_verification"] = true;
    result["apriltag_devices"] = std::vector<std::string>{"cpu", "cuda"};
#else
    result["cuda_apriltag_compiled"] = false;
    result["apriltag_devices"] = std::vector<std::string>{"cpu"};
#endif
#ifdef CUSTOM_VISION_HAVE_CUDA
    result["cuda_compiled"] = true;
    int devices = 0;
    try { devices = std::max(0, cv::cuda::getCudaEnabledDeviceCount()); } catch (const cv::Exception&) {}
    result["cuda_devices"] = devices;
    result["cuda_preprocess"] = devices > 0;
#else
    result["cuda_compiled"] = false;
    result["cuda_devices"] = 0;
    result["cuda_preprocess"] = false;
#endif
    return result;
}

PYBIND11_MODULE(_native, module) {
    // Configure this OpenCV build once before camera workers start. Python cv2
    // may load a different OpenCV build with a separate global thread pool.
    cv::setNumThreads(1);
    module.doc() = "C++ AprilTag/PnP with zero-copy gray input and optional CUDA grayscale";
    module.def("capabilities", &capabilities);
    module.def("set_opencv_threads", [](int threads) {
        if (threads < 1 || threads > 64) throw py::value_error("OpenCV threads must be between 1 and 64");
        cv::setNumThreads(threads);
    }, py::arg("threads"), "Set global native OpenCV worker limit before starting any camera workers.");
    py::class_<Detector>(module, "Detector")
        .def(py::init<const py::dict&, py::object>(), py::arg("config"), py::arg("calibration") = py::none())
        .def("process", &Detector::process, py::arg("frame").noconvert())
        .def("estimate_pose", &Detector::estimate_pose)
        .def_property_readonly("last_timings", &Detector::last_timings)
        .def_property_readonly("last_profile", &Detector::last_profile);
}
