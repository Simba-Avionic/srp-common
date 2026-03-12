#include <iostream>
#include <fstream>
#include "lib/Matrix.h"
#include "lib/KalmanFilterR7.h"
#include "lib/ApogeeDetector.h"

std::vector<double> readColumnCSV(const std::string &path, size_t colIndex)
{
    std::vector<double> data;

    try
    {
        if (path.find("accelerometer") != std::string::npos)
        {
            CsvReader<double, double, double, double> csvReader(path);
            for (size_t i = 0; i < csvReader.rowCount(); ++i)
            {
                auto row = csvReader[i];
                switch (colIndex)
                {
                case 0:
                    data.push_back(row.template get<0>());
                    break;
                case 1:
                    data.push_back(row.template get<1>());
                    break;
                case 2:
                    data.push_back(row.template get<2>());
                    break;
                case 3:
                    data.push_back(row.template get<3>());
                    break;
                default:
                    break;
                }
            }
        }
        else if (path.find("barometer") != std::string::npos)
        {
            CsvReader<double, double> csvReader(path);
            for (size_t i = 0; i < csvReader.rowCount(); ++i)
            {
                auto row = csvReader[i];
                switch (colIndex)
                {
                case 0:
                    data.push_back(row.template get<0>());
                    break;
                case 1:
                    data.push_back(row.template get<1>());
                    break;
                default:
                    break;
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading CSV file " << path << ": " << e.what() << std::endl;
    }

    return data;
}

int main(int argc, char *argv[])
{
    if (argc != 3&& argc!=1)
    {
        std::cerr << "Usage: " << argv[0];
        return 1;
    }
    std::string barometerPath;
    std::string accelerometerPath;
    if(argc==3){
        barometerPath = argv[1];
        accelerometerPath = argv[2];
    }
    if(argc==1){
        barometerPath = "data/barometer.csv";
        accelerometerPath = "data/accelerometer.csv";
    }
    KalmanFilterR7 kf(0.1);
    RealTimeApogee apogee;

    const double dt = 0.1;

    std::vector<double> pressureData = readColumnCSV(barometerPath, 1);
    std::vector<double> accelData = readColumnCSV(accelerometerPath, 3);
    if (accelData.empty() || pressureData.empty())
    {
        std::cerr << "Error: data is not loaded!\n";
        return 1;
    }
    
    size_t dataSize = accelData.size();

    for (size_t i = 0; i < dataSize; ++i)
    {
        double t = i * dt;
        double accel = accelData[i];
        double pressure = pressureData[i];

        Matrix state = kf.processMeasurement(accel, pressure);
        double height = state(0,0);
        double velocity = state(1,0);

        apogee.update(height, velocity);

        std::cout << "t=" << t << " s, h=" << height << " m, v=" << velocity << " m/s\n";

        if (apogee.isApogeeReached())
        {
            std::cout << ">>> Apogee reached: " << apogee.getApogee() << " meters\n";
            break;
        }
    }

    return 0;
}