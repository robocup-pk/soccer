#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <cmath>

namespace algos {

struct Vector2D {
    double x, y;
    
    Vector2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    
    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }
    
    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }
    
    Vector2D operator*(double scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }
    
    double length() const {
        return std::sqrt(x * x + y * y);
    }
    
    Vector2D normalized() const {
        double len = length();
        if (len < 1e-6) return Vector2D(0, 0);
        return Vector2D(x / len, y / len);
    }
    
    // Alias for compatibility with existing code
    Vector2D normalize() const {
        double len = length();
        if (len > 0.001) {
            return Vector2D(x / len, y / len);
        }
        return Vector2D(0, 0);
    }
    
    double distance(const Vector2D& other) const {
        return (*this - other).length();
    }
    
    double dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }
};

} // namespace algos

#endif // VECTOR2D_H