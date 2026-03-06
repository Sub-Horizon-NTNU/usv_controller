
#pragma once

struct Waypoint{
    double x{};
    double y{};
    double z{};
    double heading{};
    bool hold_at_point{}; // Point should be held
    double hold_time{}; // Point hold duration
};

struct States{
    float x{};
    float y{};
    float z{};
    float heading{};
    float vx{};
    float vy{};
    float angular_velocity{};
};