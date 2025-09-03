#pragma once

#include <chrono>
#include <string>
#include <iostream>

class AlgorithmTimer
{
public:
    AlgorithmTimer(const std::string& name = "") : name(name), running(false) {}

    void start() {
        startTime = std::chrono::high_resolution_clock::now();
        running = true;
    }

    void stop() {
        if (!running) return;
        endTime = std::chrono::high_resolution_clock::now();
        running = false;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        std::cout << "[Timer] " << name << " took " << duration.count() / 1000.0 << " ms" << std::endl;
    }

    // µ¥Î»: ºÁÃë
    double elapsedMilliseconds() const {
        auto now = running ? std::chrono::high_resolution_clock::now() : endTime;
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
        return duration.count() / 1000.0;
    }

private:
    std::string name;
    bool running;
    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point endTime;
};
