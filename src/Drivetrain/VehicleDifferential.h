#pragma once
#include <array>
#include <vector>
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
