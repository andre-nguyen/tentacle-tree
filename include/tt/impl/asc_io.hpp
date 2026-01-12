#pragma once

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "tt/impl/point_xyzrgb.hpp"

#include <cmath>

namespace tt {

// Represents a loaded point cloud with coordinates and per-point RGB colors
template <typename FloatT>
struct PointCloudRGB {
    std::vector<std::array<FloatT, 3>> coords;
    std::vector<std::array<std::uint8_t, 3>> colors;
};

// Loads an ASC point-cloud file where each line contains 6 whitespace-separated values:
// x y z r g b
// - x,y,z are coordinates
// - r,g,b are color channels in [0,255]
//
// Primary API: returns both coordinate and color arrays packed in PointCloudRGB<FloatT>
template <typename FloatT>
PointCloudRGB<FloatT> loadASC(const std::filesystem::path &path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("failed to open ASC file: " + path.string());
    }

    std::string line;
    std::size_t line_no = 0;
    std::vector<std::array<FloatT, 3>> coords;
    std::vector<std::array<std::uint8_t, 3>> colors;
    coords.reserve(std::pow(2, 20));
    colors.reserve(std::pow(2, 20));

    while (std::getline(in, line)) {
        ++line_no;
        auto first_non_ws = line.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) {
            continue; // empty line
        }
        if (line[first_non_ws] == '#') {
            continue; // comment
        }

        std::istringstream iss(line);
        int x, y, z;
        int r, g, b;
        if (!(iss >> x >> y >> z >> r >> g >> b)) {
            // skip malformed lines
            continue;
        }

        r = std::clamp(r, 0, 255);
        g = std::clamp(g, 0, 255);
        b = std::clamp(b, 0, 255);

        coords.push_back({static_cast<FloatT>(x), static_cast<FloatT>(y), static_cast<FloatT>(z)});
        colors.push_back({static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(g),
                          static_cast<std::uint8_t>(b)});
    }

    PointCloudRGB<FloatT> out;
    out.coords = std::move(coords);
    out.colors = std::move(colors);
    return out;
}

// Convenience overload: return vector<PointXYZRGB<FloatT>> by calling the primary API
template <typename FloatT>
std::vector<tt::PointXYZRGB<FloatT>> loadASCasPoints(const std::string &path) {
    auto pc = loadASC<FloatT>(path);
    std::vector<tt::PointXYZRGB<FloatT>> pts;
    pts.reserve(pc.coords.size());
    for (std::size_t i = 0; i < pc.coords.size(); ++i) {
        pts.emplace_back(pc.coords[i], pc.colors[i]);
    }
    return pts;
}

} // namespace tt
