#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include "CsvReader.h"

class RealTimeApogee {
public:
    // Добавим minHeight или minTime, чтобы не сработать на стартовом столе
    RealTimeApogee(size_t bufferSize = 15, double speedThreshold = -0.5)
        : bufferSize(bufferSize), speedThreshold(speedThreshold),
          maxHeight(-1e9), detectedApogee(0), apogeeReached(false),
          isLaunched(false) {}

    void update(double height, double velocity) {
        // 1. Детекция старта (чтобы не сработать, пока ракета просто стоит)
        if (!isLaunched && velocity > 5.0) { 
            isLaunched = true;
        }

        if (!isLaunched) return; // Выходим, если еще не взлетели

        heightBuffer.push_back(height);
        speedBuffer.push_back(velocity);
        
        if (heightBuffer.size() > bufferSize) {
            heightBuffer.erase(heightBuffer.begin());
            speedBuffer.erase(speedBuffer.begin());
        }

        // Обновляем максимальную высоту
        if (height > maxHeight) {
            maxHeight = height;
        }

        // 2. Условие апогея: мы взлетели И скорость стала ниже порога
        // Лучше использовать среднее по буферу, чтобы не сработать на одном "шумном" замере
        if (!apogeeReached && heightBuffer.size() >= bufferSize) {
            double avgV = 0;
            for(double v : speedBuffer) avgV += v;
            avgV /= speedBuffer.size();

            if (avgV <= speedThreshold) {
                apogeeReached = true;
                detectedApogee = maxHeight;
            }
        }
    }
    bool isApogeeReached() const { return apogeeReached; }
    double getApogee() const { return detectedApogee; }

private:
    size_t bufferSize;
    double speedThreshold;
    std::vector<double> heightBuffer;
    std::vector<double> speedBuffer;
    double maxHeight;
    double detectedApogee;
    bool apogeeReached;
    bool isLaunched;
};