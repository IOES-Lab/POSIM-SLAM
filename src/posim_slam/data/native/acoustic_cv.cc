// Bounded C ABI for the OpenCV frontend used by acoustic_flow.py.
// Numerical fitting and streaming state remain in the shared Python core.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cstdint>
#include <vector>

static bool shape(int width, int height) {
    return width > 0 && height > 0 && width <= 4096 && height <= 4096 &&
           int64_t(width) * height <= 2000000;
}
extern "C" {
int wwos_acoustic_abi() { return 1; }
const char* wwos_acoustic_opencv_version() { return CV_VERSION; }
void wwos_acoustic_single_thread() { cv::setNumThreads(1); }
int wwos_acoustic_blur(int width, int height, const uint8_t* input, uint8_t* output) {
    if (!shape(width, height) || !input || !output) return -1;
    try {
        cv::Mat a(height, width, CV_8U, const_cast<uint8_t*>(input));
        cv::Mat b(height, width, CV_8U, output);
        cv::GaussianBlur(a, b, cv::Size(5, 5), 0);
        return 0;
    } catch (...) { return -2; }
}
int wwos_acoustic_corners(int width, int height, const uint8_t* image,
                         const uint8_t* mask, float* xy, int capacity) {
    if (!shape(width, height) || !image || !mask || !xy || capacity != 300) return -1;
    try {
        cv::Mat a(height, width, CV_8U, const_cast<uint8_t*>(image));
        cv::Mat m(height, width, CV_8U, const_cast<uint8_t*>(mask));
        std::vector<cv::Point2f> points;
        cv::goodFeaturesToTrack(a, points, 300, .04, 5, m, 7);
        for (size_t i = 0; i < points.size(); ++i) {
            xy[2*i] = points[i].x; xy[2*i+1] = points[i].y;
        }
        return int(points.size());
    } catch (...) { return -2; }
}
int wwos_acoustic_flow(int width, int height, const uint8_t* previous,
                       const uint8_t* current, const float* xy, int count,
                       float* moved, uint8_t* status, float* error) {
    if (!shape(width, height) || !previous || !current || !xy || !moved ||
        !status || !error || count < 1 || count > 300) return -1;
    try {
        cv::Mat a(height, width, CV_8U, const_cast<uint8_t*>(previous));
        cv::Mat b(height, width, CV_8U, const_cast<uint8_t*>(current));
        std::vector<cv::Point2f> points(count), next;
        for (int i = 0; i < count; ++i) points[i] = cv::Point2f(xy[2*i], xy[2*i+1]);
        std::vector<uint8_t> valid;
        std::vector<float> residual;
        cv::calcOpticalFlowPyrLK(a, b, points, next, valid, residual, cv::Size(21, 21), 2);
        if (next.size() != size_t(count) || valid.size() != size_t(count) || residual.size() != size_t(count)) return -3;
        for (int i = 0; i < count; ++i) {
            moved[2*i] = next[i].x; moved[2*i+1] = next[i].y;
            status[i] = valid[i]; error[i] = residual[i];
        }
        return 0;
    } catch (...) { return -2; }
}
}
