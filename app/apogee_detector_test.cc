#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath> 
#include <filesystem> 
#include "tools/cpp/runfiles/runfiles.h" // <--- DODANY INCLUDE

#include "lib/Matrix.h"
#include "lib/KalmanFilterR7.h"
#include "lib/ApogeeDetector.h"
#include "lib/DataLoader.h"

using bazel::tools::cpp::runfiles::Runfiles; // <--- DODANA PRZESTRZEŃ NAZW
namespace fs = std::filesystem; 

namespace {
    static constexpr auto apoogeeError=0.15;
}

// Funkcja pomocnicza do pobierania bezpiecznej ścieżki
fs::path GetRunfilesPath(Runfiles* runfiles, const std::string& relative_path) {
    // Bzlmod domyślnie nazywa główne repozytorium "_main" lub tak jak w MODULE.bazel ("bazel_apogeum")
    std::string runfiles_path = "bazel_apogeum/" + relative_path; 
    fs::path resolved_path = runfiles->Rlocation(runfiles_path);

    // Fallback dla nowszych wersji Bazela, które mogą używać nazwy "_main"
    if (!fs::exists(resolved_path)) {
        runfiles_path = "_main/" + relative_path;
        resolved_path = runfiles->Rlocation(runfiles_path);
    }
    return resolved_path;
}

TEST(detect_test, test_apogee_detecotr) {
    // Inicjalizacja Runfiles
    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::CreateForTest(&error));
    ASSERT_NE(runfiles, nullptr) << "Nie udalo sie zainicjowac runfiles: " << error;

    fs::path base_path = GetRunfilesPath(runfiles.get(), "data/10hz_1000m"); 
    
    fs::path barometerPath = base_path / "exported_barometer_data.csv";
    fs::path accelerometerPath = base_path / "exported_accel_data.csv";
    fs::path configPath = base_path / "config.txt";

    checkAndGenerateConfig(configPath);
    ConfigData config = readConfig(configPath);
    const double dt = config.hz > 0 ? 1.0 / config.hz : 0.1;
    KalmanFilterR7 kf(dt);
    RealTimeApogee apogee(15, -0.5, 0.0);
    
    std::vector<double> pressureData = readColumnCSV(barometerPath, 1);
    std::vector<double> accelData = readColumnCSV(accelerometerPath, 3);
    size_t dataSize = accelData.size();
    
    double height = 0.0; 
    for (size_t i = 0; i < dataSize; ++i) {
        double accel = accelData[i];
        double pressure = pressureData[i];
        Matrix state = kf.processMeasurement(accel, pressure);
        height = state(0,0);
        double velocity = state(1,0);
        apogee.update(height, velocity);
        if (apogee.isApogeeReached()) { 
            break;
        }
    }
    EXPECT_NEAR(config.max_height_clean, height, 1.0); 
}

class ApogeeFlightDataTest : public ::testing::TestWithParam<std::string>{};

TEST_P(ApogeeFlightDataTest, CalculatesApogeeCorrectly){
    // Inicjalizacja Runfiles
    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::CreateForTest(&error));
    ASSERT_NE(runfiles, nullptr) << "Nie udalo sie zainicjowac runfiles: " << error;

    fs::path base_path = GetRunfilesPath(runfiles.get(), GetParam());

    fs::path barometerPath = base_path / "exported_barometer_data.csv";
    fs::path accelerometerPath = base_path / "exported_accel_data.csv";
    fs::path configPath = base_path / "config.txt";

    checkAndGenerateConfig(configPath);
    ConfigData config = readConfig(configPath);
    const double dt = config.hz > 0 ? 1.0 / config.hz : 0.1;
    KalmanFilterR7 kf(dt);
    RealTimeApogee apogee(15, -0.5, 0.0);
    
    std::vector<double> pressureData = readColumnCSV(barometerPath, 1);
    std::vector<double> accelData = readColumnCSV(accelerometerPath, 3);
    size_t dataSize = accelData.size();
    
    double height = 0.0; 
    for (size_t i = 0; i < dataSize; ++i) {
        double accel = accelData[i];
        double pressure = pressureData[i];
        Matrix state = kf.processMeasurement(accel, pressure);
        height = state(0,0);
        double velocity = state(1,0);
        apogee.update(height, velocity);
        if (apogee.isApogeeReached()) { 
            break;
        }
    }
    EXPECT_NEAR(config.max_height_clean, height, config.max_height_clean*apoogeeError); 
}

INSTANTIATE_TEST_SUITE_P(
    FlightDataSimulation, 
    ApogeeFlightDataTest, 
    ::testing::Values(
        "data/10hz_1000m",
        "data/10hz_2000m",
        "data/10hz_4000m",
        "data/10hz_5000m",
        "data/10hz_6000m",
        "data/100hz_1000m",
        "data/100hz_2000m",
        "data/100hz_3000m",
        "data/100hz_4000m",
        "data/100hz_5000m",
        "data/100hz_6000m",
        "data/1000hz_1000m",
        "data/1000hz_2000m",
        "data/1000hz_5000m",
        "data/1000hz_6000m"
    )
);