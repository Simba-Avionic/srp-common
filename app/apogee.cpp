#include <iostream>
#include <fstream>
#include <vector>
#include <cmath> // dodane do std::abs
#include "lib/Matrix.h"
#include "lib/KalmanFilterR7.h"
#include "lib/ApogeeDetector.h"
#include "lib/DataLoader.h"

int main(int argc, char *argv[])
{
    if (argc != 4 && argc != 1)
    {
        std::cerr << "Usage: " << argv[0] << " [barometer_file accelerometer_file config_file]\n";
        return 1;
    }

    std::string barometerPath = "data/barometer.csv";
    std::string accelerometerPath = "data/accelerometer.csv";
    std::string configPath = "data/config.txt";

    if (argc == 4) {
        barometerPath = argv[1];
        accelerometerPath = argv[2];
        configPath = argv[3];
    }

    // --- 1. Aktualizacja lub generowanie config.txt ---
    checkAndGenerateConfig(configPath);

    // --- 2. Wczytanie przygotowanego configu ---
    ConfigData config = readConfig(configPath);

    const double dt = config.hz > 0 ? 1.0 / config.hz : 0.1;
    KalmanFilterR7 kf(dt);
    RealTimeApogee apogee(15, -0.5, 0.0);

    // --- 3. Wczytanie sensorów ---
    std::vector<double> pressureData = readColumnCSV(barometerPath, 1);
    std::vector<double> accelData = readColumnCSV(accelerometerPath, 3);
    
    if (accelData.empty() || pressureData.empty())
    {
        std::cerr << "Error: data is not loaded!\n";
        return 1;
    }
    
    size_t dataSize = accelData.size();

    // --- 4. Główna pętla detekcji ---
    for (size_t i = 0; i < dataSize; ++i) {
        double t = i * dt;
        double accel = accelData[i];
        double pressure = pressureData[i];

        Matrix state = kf.processMeasurement(accel, pressure);
        double height = state(0,0);
        double velocity = state(1,0);

        apogee.update(height, velocity);

        if (apogee.isApogeeReached()) { 
            double apogeeheight = apogee.getApogee();
            std::cout << ">>> Wykryto Apogeum: " << height << " m w czasie t = " << t << " s, max height: " << apogeeheight << " m\n";

            if (config.loaded) {
                double heightError = config.max_height_clean - height;
                double heightErrorPct = (heightError / config.max_height_clean) * 100.0;
                double timeError = config.time_to_apogee - t;
                double timeErrorPct = (timeError / config.time_to_apogee) * 100.0;

                std::cout << "\n--- Porównanie z danymi referencyjnymi (Config) ---\n";
                std::cout << "Prawdziwa wys. (clean): " << config.max_height_clean << " m\n";
                std::cout << "Błąd wysokości:         " << std::abs(heightError) << " m (" << std::abs(heightErrorPct) << "%)\n";
                std::cout << "Prawdziwy czas:         " << config.time_to_apogee << " s\n";
                std::cout << "Błąd czasu detekcji:    " << std::abs(timeError) << " s (" << std::abs(timeErrorPct) << "%)\n";
            }
            break;
        }
    }

    return 0;
}