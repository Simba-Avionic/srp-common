#include <iostream>
#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include "lib/KalmanFilterR7.h"
#include "csvReader/CsvReader.h"
#include <cmath>

double pressureToAltitude(double P) {
    const double P0 = 100737.4661; 
    return 44330.0 * (1.0 - pow(P / P0, 1.0 / 5.255));
}

std::vector<double> readColumnCSV(const std::string& path, size_t colIndex) {
    std::vector<double> data;
    
    try {
        if (path.find("accelerometer") != std::string::npos) {
            CsvReader<double, double, double, double> csvReader(path);
            for (size_t i = 0; i < csvReader.rowCount(); ++i) {
                auto row = csvReader[i];
                switch (colIndex) {
                    case 0: data.push_back(row.template get<0>()); break;
                    case 1: data.push_back(row.template get<1>()); break;
                    case 2: data.push_back(row.template get<2>()); break;
                    case 3: data.push_back(row.template get<3>()); break;
                    default: break;
                }
            }
        } else if (path.find("barometer") != std::string::npos) {
            CsvReader<double, double> csvReader(path);
            for (size_t i = 0; i < csvReader.rowCount(); ++i) {
                auto row = csvReader[i];
                switch (colIndex) {
                    case 0: data.push_back(row.template get<0>()); break;
                    case 1: data.push_back(row.template get<1>()); break;
                    default: break;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading CSV file " << path << ": " << e.what() << std::endl;
    }
    
    return data;
}



int main() {
    const double dt = 0.1;
    KalmanFilterR7 kf(dt);

    std::vector<double> accelData = readColumnCSV("data/accelerometer_noisy_nosecone.csv", 3);
    std::vector<double> pressureData = readColumnCSV("data/barometer_clean.csv", 1); 

    if (accelData.empty() || pressureData.empty()) {
        return 1;
    }

    size_t dataSize = std::min(accelData.size(), pressureData.size());

    std::cout << "time;raw_accel;raw_press;kf_h;kf_v" << std::endl;

    for (size_t i = 0; i < dataSize; ++i) {
        double t = i * dt;
        double accel = accelData[i];
        double pressure = pressureData[i]; 

        double altitude_m = pressureToAltitude(pressure);
Eigen::Vector2d state = kf.processMeasurement(accel, altitude_m);
        
        std::cout << std::fixed << std::setprecision(4)
                  << t << ";" 
                  << accel << ";" 
                  << pressure << ";" 
                  << state(0) << ";"  // Height
                  << state(1) << std::endl; // Velocity
                  
        if (i > 2000) break;
    }

    return 0;
}