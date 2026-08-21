#include "safety_monitor.hpp"

#include <cmath>
#include <limits>
#include <cstdlib>

namespace arl {
namespace {

constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

bool isFinite(const Detection& detection) {
    return std::isfinite(detection.forward)
        && std::isfinite(detection.left)
        && std::isfinite(detection.confidence);
}

}

std::vector<Obstacle> processDetections(
    const std::vector<Detection>& detections,
    const RoverPose& pose,
    const SafetyConfig& config) {
    std::vector<Obstacle> obstacles;

    const double headingRadians = pose.headingDegrees * DEG_TO_RAD;
    const double cosine = std::cos(headingRadians);
    const double sine = std::sin(headingRadians);

    for (std::size_t index = 0; index < detections.size(); ++index) {
        const auto& detection = detections[index];
        const double range = std::hypot(detection.forward, detection.left);

        const bool validConfidence = detection.confidence >= config.minimumConfidence
            && detection.confidence <= 1.0;
        const bool validRange = range > 0.0 && range <= config.maximumRangeMeters;

        if (!isFinite(detection) || !validConfidence || !validRange) {
            continue;
        }

        obstacles.push_back({
            detection.id,
            detection.forward,
            detection.left,
            pose.worldX + cosine * detection.forward - sine * detection.left,
            pose.worldY + sine * detection.forward + cosine * detection.left,
            range,
        });
    }

    return obstacles;
}

std::optional<Obstacle> findNearestObstacle(const std::vector<Obstacle>& obstacles) {
    if (obstacles.empty()) {
        return std::nullopt;
    }

    const Obstacle* nearest = &obstacles.front();
    for (const auto& obstacle : obstacles) {

        if (obstacle.range < nearest->range) {
            nearest = &obstacle;
        }
    }

    return *nearest;
}

double calculateStoppingDistance(double speedKph, const SafetyConfig& config) {

    const double speedMps = speedKph / 3.6;
    const double reactionDistance = speedMps * config.reactionTimeSeconds;
    const double brakingDistance = (speedMps * speedMps)
        / (2.0 * config.maximumDecelerationMps2);
    return reactionDistance + brakingDistance;
}

bool shouldEmergencyBrake(const std::vector<Obstacle>& obstacles, double speedKph, const SafetyConfig& config) {
    const double stoppingDistance = calculateStoppingDistance(speedKph, config);

    for (const auto& obstacle : obstacles) {
        const bool isAhead = obstacle.forward >= 0.0;
        const bool isInLane = std::abs(obstacle.left) <= config.laneHalfWidthMeters;
        const bool isWithinStoppingDist = obstacle.forward <= stoppingDistance;

        if (isAhead && isInLane && isWithinStoppingDist) {
            return true;
        }
    }

    return false;
}

}
