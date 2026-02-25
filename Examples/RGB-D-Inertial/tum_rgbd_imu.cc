/**
 * Offline TUM-RGBD + IMU player for ORB-SLAM3 (IMU_RGBD mode)
 *
 * Expected structure:
 *  dataset_root/
 *    rgb/...
 *    depth/...
 *    association.txt  (recommended)
 *    imu.csv
 *
 * association.txt format (recommended, typical):
 *   t_rgb rgb/xxx.png t_depth depth/xxx.png
 *
 * imu.csv format (you decide, but this reader supports common patterns):
 *   t,wx,wy,wz,ax,ay,az
 * or
 *   t ax ay az wx wy wz
 *
 * Units:
 *  - timestamps in seconds
 *  - gyro in rad/s
 *  - accel in m/s^2
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <System.h>

using namespace std;

struct RGBDFrame
{
    double t;              // frame timestamp (seconds)
    std::string rgb_path;  // relative or absolute
    std::string depth_path;
};

struct IMUSample
{
    double t;     // seconds
    double ax, ay, az; // m/s^2
    double wx, wy, wz; // rad/s
};

// ---------- helpers ----------
static inline bool starts_with(const std::string& s, const std::string& p) {
    return s.rfind(p, 0) == 0;
}

static std::string join_path(const std::string& root, const std::string& rel)
{
    if (rel.empty()) return root;
    if (!root.empty() && root.back() == '/') return root + rel;
    return root + "/" + rel;
}

static bool parse_association_line(const std::string& line, RGBDFrame& out)
{
    // Skip comments/empty
    if (line.empty()) return false;
    if (starts_with(line, "#")) return false;

    std::istringstream iss(line);
    double t_rgb = 0.0, t_depth = 0.0;
    std::string rgb, depth;

    // Typical association.txt: t_rgb rgb_path t_depth depth_path
    if (!(iss >> t_rgb >> rgb >> t_depth >> depth)) return false;

    // Choose a single timestamp for TrackRGBD. Usually t_rgb is fine if aligned;
    // if you want, you can average them if they differ slightly.
    out.t = t_rgb;
    out.rgb_path = rgb;
    out.depth_path = depth;
    return true;
}

static std::vector<RGBDFrame> load_association(const std::string& dataset_root,
                                               const std::string& assoc_file)
{
    std::ifstream f(assoc_file);
    if (!f.is_open())
        throw std::runtime_error("Cannot open association file: " + assoc_file);

    std::vector<RGBDFrame> frames;
    std::string line;
    while (std::getline(f, line))
    {
        RGBDFrame fr;
        if (parse_association_line(line, fr))
        {
            // Make absolute paths if needed
            fr.rgb_path   = join_path(dataset_root, fr.rgb_path);
            fr.depth_path = join_path(dataset_root, fr.depth_path);
            frames.push_back(fr);
        }
    }

    if (frames.empty())
        throw std::runtime_error("No frames parsed from association file.");

    // Ensure sorted by timestamp
    std::sort(frames.begin(), frames.end(),
              [](const RGBDFrame& a, const RGBDFrame& b){ return a.t < b.t; });

    return frames;
}

static std::vector<IMUSample> downsample_imu(const std::vector<IMUSample>& in, double target_hz)
{
    std::vector<IMUSample> out;
    out.reserve(in.size());
    const double min_dt = 1.0 / target_hz;

    double last_t = -1e100;
    for (const auto& s : in)
    {
        if (s.t - last_t >= min_dt * 0.999) // ほぼ等間隔に
        {
            out.push_back(s);
            last_t = s.t;
        }
    }
    return out;
}

static bool parse_imu_line_guess_format(const std::string& line, IMUSample& out)
{
    // Skip comments/empty
    if (line.empty()) return false;
    if (starts_with(line, "#")) return false;

    // Accept comma-separated or whitespace-separated
    std::string s = line;
    for (char& c : s) if (c == ',') c = ' ';

    std::istringstream iss(s);

    // Try to parse 7 numbers: t + 6 axes
    double t,a,b,c,d,e,f;
    if (!(iss >> t >> a >> b >> c >> d >> e >> f)) return false;

    out.t = t;
    out.wx = a; out.wy = b; out.wz = c;
    out.ax = d; out.ay = e; out.az = f;
    return true;
}

static std::vector<IMUSample> load_imu(const std::string& imu_csv_path)
{
    std::ifstream f(imu_csv_path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open IMU file: " + imu_csv_path);

    std::vector<IMUSample> imu;
    std::string line;

    while (std::getline(f, line))
    {
        IMUSample s;
        if (parse_imu_line_guess_format(line, s))
            imu.push_back(s);
    }

    if (imu.empty())
        throw std::runtime_error("No IMU samples parsed from: " + imu_csv_path);

    std::sort(imu.begin(), imu.end(),
              [](const IMUSample& a, const IMUSample& b){ return a.t < b.t; });

    return imu;
}

// For each frame time t_k, collect IMU samples (t_{k-1}, t_k] and convert to ORB-SLAM3 points.
static void collect_imu_segment(const std::vector<IMUSample>& imu,
                                size_t& imu_idx,           // moving cursor into imu[]
                                double t_prev, double t_curr,
                                std::vector<ORB_SLAM3::IMU::Point>& out_points)
{
    out_points.clear();

    // Advance imu_idx to first sample with t > t_prev
    while (imu_idx < imu.size() && imu[imu_idx].t <= t_prev)
        imu_idx++;

    // Collect until t <= t_curr
    while (imu_idx < imu.size() && imu[imu_idx].t <= t_curr)
    {
        const auto& s = imu[imu_idx];

        // ORB_SLAM3::IMU::Point expects: ax,ay,az, wx,wy,wz, timestamp
        out_points.emplace_back(
            s.ax, s.ay, s.az,
            s.wx, s.wy, s.wz,
            s.t
        );
        imu_idx++;
    }
}

int main(int argc, char** argv)
{
    // Usage:
    // ./tum_rgbd_imu path_to_vocabulary path_to_settings dataset_root [trajectory_file_name]
    if (argc < 4 || argc > 5)
    {
        std::cerr << "\nUsage: " << argv[0]
                  << " path_to_vocabulary path_to_settings dataset_root (trajectory_file_name)\n";
        return 1;
    }

    const std::string vocab_path    = argv[1];
    const std::string settings_path = argv[2];
    const std::string dataset_root  = argv[3];

    std::string traj_name;
    if (argc == 5) traj_name = argv[4];

    const std::string assoc_path = join_path(dataset_root, "association.txt");
    const std::string imu_path   = join_path(dataset_root, "imu.csv");

    // Load dataset
    std::vector<RGBDFrame> frames = load_association(dataset_root, assoc_path);
    std::vector<IMUSample> imu    = load_imu(imu_path);
    // imu = downsample_imu(imu, 200.0);

    // Create SLAM system (IMU_RGBD)
    ORB_SLAM3::System SLAM(vocab_path, settings_path, ORB_SLAM3::System::IMU_RGBD,
                           true, 0, traj_name);

    const float imageScale = SLAM.GetImageScale();

    // Playback
    size_t imu_idx = 0;
    double t_prev = frames.front().t;

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for (size_t k = 0; k < frames.size() && !SLAM.isShutDown(); ++k)
    {
        const auto& fr = frames[k];
        const double t = fr.t;

        // Load images
        cv::Mat im = cv::imread(fr.rgb_path, cv::IMREAD_UNCHANGED);
        if (im.empty())
        {
            std::cerr << "Failed to load RGB: " << fr.rgb_path << "\n";
            continue;
        }

        // ORB-SLAM3 commonly expects RGB or BGR depending on config.
        // TUM RGB images are usually in RGB? OpenCV loads color as BGR by default.
        // If your settings expect RGB, convert here:
        // cv::cvtColor(im, im, cv::COLOR_BGR2RGB);

        cv::Mat depth = cv::imread(fr.depth_path, cv::IMREAD_UNCHANGED);
        if (depth.empty())
        {
            std::cerr << "Failed to load depth: " << fr.depth_path << "\n";
            continue;
        }

        // TUM depth is often 16UC1 in millimeters (dataset-dependent).
        // ORB-SLAM3 uses depthScale from settings, so keep raw 16U unless you know you need conversion.

        // Collect IMU segment between previous and current frame
        if (k == 0) t_prev = t; // no prior interval
        collect_imu_segment(imu, imu_idx, t_prev, t, vImuMeas);

        // Resize if needed
        if (imageScale != 1.f)
        {
            int width  = int(im.cols * imageScale);
            int height = int(im.rows * imageScale);
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
        }

        static double last_t = -1;
        if (last_t > 0) {
          std::cerr << std::fixed << std::setprecision(9)
                    << "t=" << t << " dt=" << (t - last_t)
                    << " imu_sent=" << vImuMeas.size() << "\n";
        }
        last_t = t;

        // Track
        SLAM.TrackRGBD(im, depth, t, vImuMeas);

        t_prev = t;
    }

    std::cout << "Finished dataset playback.\n";
    return 0;
}