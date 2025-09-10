#include "DestinationForTimedPositionCalc.h"
#include <cmath>
#include <stdexcept>
#include <Eigen/Geometry>

// Local constants (to avoid circular dependency with BangBangTrajectoryFactory)
namespace {
    const float SYNC_ACCURACY = 1e-3f;
    const std::function<float(float)> ALPHA_FN_ASYNC = 
        [](float alpha) { 
            return alpha + (((float)M_PI_2 - alpha) * 0.5f); 
        };
}

// Corresponds to DestinationForTimedPositionCalc.java

Eigen::Vector2d DestinationForTimedPositionCalc::destinationForBangBang2dSync(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1, const Eigen::Vector2d& v0, double vMax, double aMax, double targetTime) {
    return destinationForBangBang2D(s0, s1, v0, static_cast<float>(vMax), static_cast<float>(aMax), static_cast<float>(targetTime), [](float a){ return a; });
}

Eigen::Vector2d DestinationForTimedPositionCalc::destinationForBangBang2dAsync(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1, const Eigen::Vector2d& v0, double vMax, double aMax, double targetTime, const Eigen::Vector2d& primaryDirection) {
    
    const double rotation = std::atan2(primaryDirection.y(), primaryDirection.x());
    Eigen::Rotation2Dd rot(-rotation);
	auto startToTarget = rot * (s1 - s0);
	auto v0Rotated = rot * v0;

    auto dest = destinationForBangBang2D(
        Eigen::Vector2d::Zero(),
        startToTarget,
        v0Rotated,
        static_cast<float>(vMax),
        static_cast<float>(aMax),
        static_cast<float>(targetTime),
        ALPHA_FN_ASYNC
    );
    
    Eigen::Rotation2Dd invRot(rotation);
    return invRot * dest + s0;
}


Eigen::Vector2d DestinationForTimedPositionCalc::destinationForBangBang2D(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1, const Eigen::Vector2d& v0, float vMax, float aMax, float targetTime, const std::function<float(float)>& alphaFn) {
    
    auto v0x = static_cast<float>(v0.x());
    auto v0y = static_cast<float>(v0.y());
    auto distance = s1 - s0;
    auto distanceX = static_cast<float>(distance.x());
    auto distanceY = static_cast<float>(distance.y());

    float inc = static_cast<float>(M_PI) / 8.0f;
    float alpha = static_cast<float>(M_PI) / 4.0f;

    TimedPos1D x = {0, 0};
    TimedPos1D y = {0, 0};

    while (inc > 1e-7) {
        const float sA = std::sin(alphaFn(alpha));
        const float cA = std::cos(alphaFn(alpha));
        
        x = getTimedPos1D(distanceX, v0x, vMax * cA, aMax * cA, targetTime);
        y = getTimedPos1D(distanceY, v0y, vMax * sA, aMax * sA, targetTime);

        double diff = std::abs(x.time - y.time);
        if (diff < SYNC_ACCURACY) {
            break;
        }
        if (x.time > y.time) {
            alpha -= inc;
        } else {
            alpha += inc;
        }
        inc *= 0.5f;
    }
    return Eigen::Vector2d(x.pos + s0.x(), y.pos + s0.y());
}


DestinationForTimedPositionCalc::TimedPos1D DestinationForTimedPositionCalc::getTimedPos1D(float s, float v0, float vMax, float aMax, float tt) {
    auto aDec = v0 >= 0 ? -aMax : aMax;
    auto sZeroVel = (aDec != 0) ? (0.5f * v0 * (-v0 / aDec)) : 0.0f;
    auto v1Max = s >= 0 ? vMax : -vMax;

    bool condition1 = (s >= 0.f) != (v0 > 0.f);
    bool condition2 = (s >= 0) == (sZeroVel < s);
    bool condition3 = calcSlowestDirectTime(s, v0, aMax) >= tt;

    if (condition1 || condition2 || condition3) {
        return calcFastestDirect(s, v0, v1Max, aMax, tt);
    } else {
        auto tBreaking = (aMax != 0) ? std::abs(v0 / aMax) : 0.0f;
        auto timed = calcFastestDirect(s - sZeroVel, 0.f, -v1Max, aMax, tt - tBreaking);
        return {timed.pos + sZeroVel, timed.time + tBreaking};
    }
}

float DestinationForTimedPositionCalc::calcSlowestDirectTime(float s, float v0, float aMax) {
    auto aDec = (v0 >= 0) ? -aMax : aMax;
    if (std::abs(aDec) < 1e-9) return std::numeric_limits<float>::infinity();
    auto sqrtVal = v0 * v0 + 2 * aDec * s;
    if (sqrtVal < 0) return std::numeric_limits<float>::infinity();
    auto sqrtRes = std::sqrt(sqrtVal);
    return (v0 >= 0.f) ? ((-v0 + sqrtRes) / aDec) : ((-v0 - sqrtRes) / aDec);
}

DestinationForTimedPositionCalc::TimedPos1D DestinationForTimedPositionCalc::calcFastestDirect(float s, float v0, float v1Max, float aMax, float tt) {
    auto aDec = v1Max >= 0 ? -aMax : aMax;
    auto trapezoidal = calcFastestDirectTrapezoidal(s, v0, v1Max, aMax, aDec, tt);
    if (trapezoidal) {
        return *trapezoidal;
    }
    return calcFastestDirectTriangular(s, v0, v1Max, aMax, aDec, tt);
}

std::optional<DestinationForTimedPositionCalc::TimedPos1D> DestinationForTimedPositionCalc::calcFastestDirectTrapezoidal(float s, float v0, float v1Max, float aMax, float aDec, float tt) {
    auto aAcc = v0 >= v1Max ? -aMax : aMax;
    if (std::abs(aAcc) < 1e-9) return std::nullopt;

    auto t01 = (v1Max - v0) / aAcc;
    auto s01 = 0.5f * (v1Max + v0) * t01;

    if ((s >= 0.0f) == (s <= s01)) {
        return std::nullopt;
    }

    auto s13 = s - s01;
    if (std::abs(aDec) < 1e-9) return std::nullopt;
    auto t23 = -v1Max / aDec;
    auto s23 = 0.5f * v1Max * t23;
    
    if (std::abs(v1Max) < 1e-9) return std::nullopt;
    auto t12TooSlow = s13 / v1Max;
    if (t01 + t12TooSlow >= tt) {
        return TimedPos1D{s + s23, t01 + t12TooSlow + t23};
    }

    auto s12Early = s13 - s23;
    auto t12Early = s12Early / v1Max;
    if (t12Early >= 0.0f && t01 + t12Early + t23 <= tt) {
        return TimedPos1D{s, t01 + t12Early + t23};
    }
    
    auto t13 = tt - t01;
    auto sqrtVal = 2 * (s13 - t13 * v1Max) / aDec;
    if (sqrtVal < 0) return std::nullopt;
    auto t23Direct = std::sqrt(sqrtVal);
    auto t12Direct = t13 - t23Direct;
    if (t12Direct > 0 && t23Direct < t23) {
        auto v3 = v1Max + aDec * t23Direct;
        auto t34 = -v3 / aDec;
        return TimedPos1D{s + 0.5f * v3 * t34, tt + t34};
    }
    return std::nullopt;
}

DestinationForTimedPositionCalc::TimedPos1D DestinationForTimedPositionCalc::calcFastestDirectTriangular(float s, float v0, float v1Max, float aMax, float aDec, float tt) {
    if ((v1Max >= 0) == (v0 >= v1Max)) {
        if (std::abs(aDec) < 1e-9) return {s, tt};
        auto t = -v0 / aDec;
        return {0.5f * v0 * t, t};
    }
    auto aAcc = -aDec;
    if (std::abs(aAcc) < 1e-9) return {s, tt};
    
    auto sqrtValTooSlow = 2 * aAcc * s + v0 * v0;
    if (sqrtValTooSlow < 0) sqrtValTooSlow = 0;
    auto sqrtTooSlow = std::sqrt(sqrtValTooSlow);
    auto t01TooSlow = (v1Max >= 0.f) ? ((-v0 + sqrtTooSlow) / aAcc) : ((-v0 - sqrtTooSlow) / aAcc);

    if (t01TooSlow >= tt) {
        auto v1TooSlow = v0 + aAcc * t01TooSlow;
        auto t12TooSlow = std::abs(v1TooSlow / aAcc);
        return {s + 0.5f * v1TooSlow * t12TooSlow, t01TooSlow + t12TooSlow};
    }

    auto sqEarlyVal = ((s * aAcc) + (0.5f * v0 * v0)) / (aMax * aMax);
    auto t12Early = sqEarlyVal > 0.0f ? std::sqrt(sqEarlyVal) : 0.0f;
    auto v1Early = aAcc * t12Early;
    auto t01Early = (v1Early - v0) / aAcc;
    if (t01Early + t12Early <= tt) {
        return {s, t01Early + t12Early};
    }

    auto sqDirectVal = 2 * aAcc * (aAcc * tt * tt - 2 * s + 2 * tt * v0);
    if (sqDirectVal < 0) sqDirectVal = 0;
    auto sqDirect = std::sqrt(sqDirectVal);
    auto t01Direct = tt - sqDirect / (2 * aMax);
    auto v1Direct = v0 + aAcc * t01Direct;
    auto t13Direct = v1Direct / aAcc;
    auto s01Direct = 0.5f * (v0 + v1Direct) * t01Direct;
    auto s13Direct = 0.5f * v1Direct * t13Direct;
    return {s01Direct + s13Direct, t01Direct + t13Direct};
}
