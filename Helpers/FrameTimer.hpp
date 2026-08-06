#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

// ──────────────────────────────────────────────────────────────────────────
// Lightweight per-stage timing helper for the console "[ DEBUG ][ DURATION ]"
// output. This replaces the old durationArr[20]/durationHeader[20] pair,
// which required every call site and the header array to stay in sync by
// hand (add/remove/reorder a stage anywhere and the numbers silently line
// up with the wrong label).
//
// - Stages are identified by name, so there's nothing to keep in sync.
// - `enabled` is a single bool (wired to the "voxel3d.enable_timing_debug"
//   ROS param in the constructor). When false, mark()/markFrom()/record()
//   are just an `if (!enabled) return;` -- no clock reads, no map lookups,
//   effectively free to leave in the code permanently.
// ──────────────────────────────────────────────────────────────────────────
class FrameTimer {
  public:
    bool enabled = true;

    // Call once at the very start of the frame being timed.
    void startFrame() {
        if (!enabled) return;
        m_last = std::chrono::steady_clock::now();
    }

    // Time elapsed since the previous mark() (or startFrame()).
    void mark(const std::string& name) {
        if (!enabled) return;
        auto now = std::chrono::steady_clock::now();
        add(name, std::chrono::duration<double, std::milli>(now - m_last).count());
        m_last = now;
    }

    // Time elapsed since an arbitrary point in time (e.g. a timestamp
    // captured on another thread), without disturbing the mark() chain.
    void markFrom(const std::string& name, std::chrono::steady_clock::time_point from) {
        if (!enabled) return;
        add(name, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - from).count());
    }

    // Record an already-computed duration directly -- no clock read at all.
    void record(const std::string& name, double ms) {
        if (!enabled) return;
        add(name, ms);
    }

    // Call once per frame. Every `windowSize` frames, prints the rolling
    // average for each stage (in the order first seen) and resets.
    void endFrame(int windowSize = 10) {
        if (!enabled) return;
        if (++m_frameCount < windowSize) return;

        std::cout << "[ DEBUG ][ DURATION ] --------------------------------" << std::endl;
        for (const auto& name : m_order) {
            std::cout << "[ DEBUG ][ DURATION ] " << name << " [  " << (m_sums[name] / windowSize) << "]" << std::endl;
            m_sums[name] = 0.0;
        }
        m_frameCount = 0;
    }

  private:
    void add(const std::string& name, double ms) {
        auto it = m_sums.find(name);
        if (it == m_sums.end()) {
            m_order.push_back(name);
            m_sums.emplace(name, ms);
        } else {
            it->second += ms;
        }
    }

    std::chrono::steady_clock::time_point m_last;
    std::unordered_map<std::string, double> m_sums;
    std::vector<std::string> m_order;
    int m_frameCount = 0;
};
