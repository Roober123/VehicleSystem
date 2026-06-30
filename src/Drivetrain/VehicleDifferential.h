#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include "wheel.h"

using std::array;
using std::vector;

class BaseDifferential {
protected:
    array<float, 4> torque_distribution{}; // max 4 wheels on an axle (usually 2)
public:
    virtual const array<float, 4>& update(const vector<godot::Wheel*>& wheels) = 0;
    virtual ~BaseDifferential() = default;
};

class Open_Differential : public BaseDifferential { // works with arbitrary number of wheels (<= 4)
public:
    const array<float, 4>& update(const vector<godot::Wheel*>& wheels) override;
};

class LSD_Differential : public BaseDifferential { // for 2 wheels
    float bias_ratio = 2.5f;
    float preload = 50.0f;
public:
    LSD_Differential() = default;

    const array<float, 4>& update(const vector<godot::Wheel*>& wheels) override;

    void set_bias_ratio(float value) { bias_ratio = std::max(1.0f, value); }
    float get_bias_ratio() const { return bias_ratio; }
    void set_preload(float value) { preload = std::max(0.0f, value); }
    float get_preload() const { return preload; }
};

class Torsen_Differential : public BaseDifferential { // for 2 wheels
    float torque_bias_ratio = 3.0f;
public:
    Torsen_Differential() = default;

    const array<float, 4>& update(const vector<godot::Wheel*>& wheels) override;

    void set_torque_bias_ratio(float value) { torque_bias_ratio = std::max(1.0f, value); }
    float get_torque_bias_ratio() const { return torque_bias_ratio; }
};