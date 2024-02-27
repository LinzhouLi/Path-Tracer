/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>

std::string timeString(double time, bool precise) {
    if (std::isnan(time) || std::isinf(time))
        return "inf";

    std::string suffix = "ms";
    if (time > 1000) {
        time /= 1000; suffix = "s";
        if (time > 60) {
            time /= 60; suffix = "m";
            if (time > 60) {
                time /= 60; suffix = "h";
                if (time > 12) {
                    time /= 12; suffix = "d";
                }
            }
        }
    }

    std::ostringstream os;
    os << std::setprecision(precise ? 4 : 1) << std::fixed << time << suffix;

    return os.str();
}

namespace pt {

/**
 * \brief Simple timer with millisecond precision
 *
 * This class is convenient for collecting performance data
 */
    class Timer {
    public:
        /// Create a new timer and reset it
        Timer() { reset(); }

        /// Reset the timer to the current time
        void reset() { start = std::chrono::system_clock::now(); }

        /// Return the number of milliseconds elapsed since the timer was last reset
        double elapsed() const {
            auto now = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            return (double)duration.count();
        }

        /// Like \ref elapsed(), but return a human-readable string
        std::string elapsedString(bool precise = false) const {
            return timeString(elapsed(), precise);
        }

        /// Return the number of milliseconds elapsed since the timer was last reset and then reset it
        double lap() {
            auto now = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            start = now;
            return (double)duration.count();
        }

        /// Like \ref lap(), but return a human-readable string
        std::string lapString(bool precise = false) {
            return timeString(lap(), precise);
        }
    private:
        std::chrono::system_clock::time_point start;
};

}