#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "motor_controller.hpp"
#include "lidar_processor.hpp"

// Lightweight test framework without external dependencies
static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (at line " << __LINE__ << ")\n"; \
            return false; \
        } \
    } while(0)

#define RUN_TEST(fn) \
    do { \
        g_tests_run++; \
        std::cout << "[RUN]  " << #fn << " ... "; \
        if (fn()) { \
            g_tests_passed++; \
            std::cout << "PASSED\n"; \
        } else { \
            std::cout << "FAILED\n"; \
        } \
    } while(0)

// Test 1: PID reaches target within tolerance
bool test_pid_convergence() {
    rover::MotorPIDController pid(50.0, 0.0, 0.5, 0.1, 0.05);
    bool stabilized = pid.run_until_stabilized(0.1, 100, false);
    TEST_ASSERT(stabilized, "PID did not stabilize within 100 iterations");
    TEST_ASSERT(std::abs(pid.getCurrentSpeed() - 50.0) <= 0.1, "Speed not within tolerance");
    return true;
}

// Test 2: PID handles deceleration and negative speeds
bool test_pid_deceleration() {
    rover::MotorPIDController pid(15.0, 60.0, 0.5, 0.1, 0.05);
    bool stabilized = pid.run_until_stabilized(0.1, 100, false);
    TEST_ASSERT(stabilized, "PID deceleration did not stabilize");
    TEST_ASSERT(std::abs(pid.getCurrentSpeed() - 15.0) <= 0.1, "Speed not within tolerance");
    return true;
}

// Test 3: Anti-windup clamping prevents runaway integral
bool test_pid_anti_windup() {
    rover::MotorPIDController pid(100.0, 0.0, 0.1, 0.5, 0.0, 20.0, 10.0);
    // Step repeatedly with large error
    for (int i = 0; i < 50; ++i) {
        pid.step(1.0);
    }
    TEST_ASSERT(std::abs(pid.getIntegralSum()) <= 20.0, "Integral exceeded anti-windup clamp limit");
    return true;
}

// Test 4: Runaway safety iteration limit
bool test_pid_safety_limit() {
    // Highly unstable/oscillatory gains
    rover::MotorPIDController pid(50.0, 0.0, 5.0, 0.0, 0.0, 50.0, 100.0);
    bool stabilized = pid.run_until_stabilized(0.0001, 15, false);
    // Should terminate gracefully after 15 iterations rather than hanging forever
    TEST_ASSERT(!stabilized, "Unstable controller should report non-convergence");
    return true;
}

// Test 5: LiDAR filters noise and negative values
bool test_lidar_noise_filtering() {
    rover::LidarProcessor proc(3.0, 6.0, 0.1, 50.0);
    std::vector<double> scan = {-5.0, 0.0, 0.02, -0.1, 8.5, 12.0};
    rover::ObstacleStatus status = proc.processScan(scan);
    TEST_ASSERT(status == rover::ObstacleStatus::CLEAR, "Filtered scan should be CLEAR");
    TEST_ASSERT(std::abs(proc.getClosestDistance() - 8.5) < 1e-4, "Closest object should be 8.5m");
    return true;
}

// Test 6: LiDAR emergency stop threshold (< 3.0m)
bool test_lidar_emergency_stop() {
    rover::LidarProcessor proc(3.0, 6.0, 0.1, 50.0);
    std::vector<double> scan = {12.5, 15.0, -1.0, 2.2, 20.0, 0.05, 5.0};
    rover::ObstacleStatus status = proc.processScan(scan);
    TEST_ASSERT(status == rover::ObstacleStatus::EMERGENCY_STOP, "Should trigger EMERGENCY_STOP");
    TEST_ASSERT(std::abs(proc.getClosestDistance() - 2.2) < 1e-4, "Closest object should be 2.2m");
    return true;
}

// Test 7: LiDAR caution zone (3.0m - 6.0m)
bool test_lidar_caution_zone() {
    rover::LidarProcessor proc(3.0, 6.0, 0.1, 50.0);
    std::vector<double> scan = {10.0, 4.5, 7.8, 12.0};
    rover::ObstacleStatus status = proc.processScan(scan);
    TEST_ASSERT(status == rover::ObstacleStatus::CAUTION, "Should trigger CAUTION");
    TEST_ASSERT(std::abs(proc.getClosestDistance() - 4.5) < 1e-4, "Closest object should be 4.5m");
    return true;
}

// Test 8: Empty or all-invalid scan handling
bool test_lidar_no_data() {
    rover::LidarProcessor proc(3.0, 6.0, 0.1, 50.0);
    std::vector<double> empty_scan;
    TEST_ASSERT(proc.processScan(empty_scan) == rover::ObstacleStatus::NO_DATA, "Empty scan should return NO_DATA");

    std::vector<double> invalid_scan = {-1.0, 0.0, 0.01, 100.0};
    TEST_ASSERT(proc.processScan(invalid_scan) == rover::ObstacleStatus::NO_DATA, "All-invalid scan should return NO_DATA");
    return true;
}

int main() {
    std::cout << "======================================\n";
    std::cout << " Rover C++ Core Logic Unit Tests\n";
    std::cout << "======================================\n";

    RUN_TEST(test_pid_convergence);
    RUN_TEST(test_pid_deceleration);
    RUN_TEST(test_pid_anti_windup);
    RUN_TEST(test_pid_safety_limit);
    RUN_TEST(test_lidar_noise_filtering);
    RUN_TEST(test_lidar_emergency_stop);
    RUN_TEST(test_lidar_caution_zone);
    RUN_TEST(test_lidar_no_data);

    std::cout << "======================================\n";
    std::cout << " Result: " << g_tests_passed << "/" << g_tests_run << " tests passed.\n";
    std::cout << "======================================\n";

    return (g_tests_passed == g_tests_run) ? 0 : 1;
}
