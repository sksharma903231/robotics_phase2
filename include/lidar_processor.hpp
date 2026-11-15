#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

namespace rover {

enum class ObstacleStatus {
    CLEAR,
    CAUTION,
    EMERGENCY_STOP,
    NO_DATA
};

/**
 * @brief Processes LiDAR range scans, filters noise/out-of-range readings,
 * and determines obstacle avoidance hazard status.
 */
class LidarProcessor {
private:
    double emergency_threshold_{3.0}; // meters
    double caution_threshold_{6.0};   // meters
    double min_valid_range_{0.1};     // meters
    double max_valid_range_{50.0};    // meters

    double closest_distance_{std::numeric_limits<double>::infinity()};
    int closest_index_{-1};
    ObstacleStatus status_{ObstacleStatus::NO_DATA};

public:
    LidarProcessor(double emergency_threshold = 3.0,
                   double caution_threshold = 6.0,
                   double min_valid_range = 0.1,
                   double max_valid_range = 50.0)
        : emergency_threshold_(emergency_threshold),
          caution_threshold_(caution_threshold),
          min_valid_range_(min_valid_range),
          max_valid_range_(max_valid_range) {}

    /**
     * @brief Processes a list of LiDAR range measurements.
     * @param ranges Vector of distance readings in meters.
     * @return Evaluated ObstacleStatus.
     */
    ObstacleStatus processScan(const std::vector<double>& ranges) {
        closest_distance_ = std::numeric_limits<double>::infinity();
        closest_index_ = -1;

        if (ranges.empty()) {
            status_ = ObstacleStatus::NO_DATA;
            return status_;
        }

        for (size_t i = 0; i < ranges.size(); ++i) {
            double r = ranges[i];

            // Filter out NaN, Inf, and out-of-sensor-bound noise
            if (std::isnan(r) || std::isinf(r)) {
                continue;
            }
            if (r < min_valid_range_ || r > max_valid_range_) {
                continue;
            }

            if (r < closest_distance_) {
                closest_distance_ = r;
                closest_index_ = static_cast<int>(i);
            }
        }

        if (std::isinf(closest_distance_)) {
            status_ = ObstacleStatus::NO_DATA;
        } else if (closest_distance_ < emergency_threshold_) {
            status_ = ObstacleStatus::EMERGENCY_STOP;
        } else if (closest_distance_ < caution_threshold_) {
            status_ = ObstacleStatus::CAUTION;
        } else {
            status_ = ObstacleStatus::CLEAR;
        }

        return status_;
    }

    double getClosestDistance() const { return closest_distance_; }
    int getClosestIndex() const { return closest_index_; }
    ObstacleStatus getStatus() const { return status_; }

    std::string getStatusString() const {
        switch (status_) {
            case ObstacleStatus::EMERGENCY_STOP:
                return "CRITICAL: Obstacle too close, brakes applied!";
            case ObstacleStatus::CAUTION:
                return "WARNING: Obstacle detected within caution zone.";
            case ObstacleStatus::CLEAR:
                return "Path is Clear.";
            case ObstacleStatus::NO_DATA:
            default:
                return "NO VALID SENSOR DATA.";
        }
    }
};

} // namespace rover
