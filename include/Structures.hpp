
#pragma once

struct Waypoint{
    double x{}; // m
    double y{}; // m
    double z{}; // m
    double heading{}; //rad
    bool hold_at_point{}; // Point should be held
    double hold_time{}; // Point hold duration (seconds)
};

struct States{
    float x{}; //m
    float y{}; //m
    float z{}; //m
    float heading{}; //Rad
    float vx{};// m/s
    float vy{};// m/s
    float angular_velocity{}; // rad/s
};