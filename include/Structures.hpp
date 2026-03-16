
#pragma once

struct States{
    float x{}; //m
    float y{}; //m
    float z{}; //m
    float heading{}; //Rad
    float vx{};// m/s
    float vy{};// m/s
    float angular_velocity{}; // rad/s
};

struct ControlCmd {
    float x{};
    float y{};
    float heading{};
    bool heading_on_path{};
    bool hold_position{};
    bool brake{};
};
