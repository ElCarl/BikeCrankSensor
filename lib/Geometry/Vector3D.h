#pragma once

#include <math.h>

template <typename T>
class Vector3D {
  public:
    constexpr Vector3D(T x, T y, T z) : _x(x), _y(y), _z(z) {}

    constexpr float magnitude() const {
        T sum_sq{(_x * _x) + (_y * _y) + (_z * _z)};
        return std::sqrt(sum_sq);
    }

    constexpr T x() const { return _x; }
    constexpr T y() const { return _y; }
    constexpr T z() const { return _z; }

  private:
    T _x;
    T _y;
    T _z;
};
