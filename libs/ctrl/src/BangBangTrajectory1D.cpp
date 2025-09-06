#include "BangBangTrajectory1D.h"
#include <algorithm>
#include <stdexcept>
#include <numeric>

namespace ctrl {

BangBangTrajectory1D::BangBangTrajectory1D() {
    for (int i = 0; i < MAX_PARTS; i++) {
        parts[i] = BBTrajectoryPart{};
    }
}

double BangBangTrajectory1D::getPosition(double tt) const {
    float trajTime = std::max(0.0f, (float)tt);
    
    if (trajTime >= getTotalTime()) {
        // requested time beyond final element
        BBTrajectoryPart lastPart = parts[numParts - 1];
        const float t = lastPart.tEnd - parts[numParts - 2].tEnd;
        return (double)(lastPart.s0 + (lastPart.v0 * t) + (0.5f * lastPart.acc * t * t));
    }
    
    auto pieceIdx = findPartIdx(trajTime);
    auto piece = parts[pieceIdx];
    auto tPieceStart = pieceIdx < 1 ? 0 : parts[pieceIdx - 1].tEnd;
    auto t = trajTime - tPieceStart;
    return (double)(piece.s0 + (piece.v0 * t) + (0.5f * piece.acc * t * t));
}

double BangBangTrajectory1D::getPositionMM(double t) const {
    return getPosition(t) * 1000.0;
}

double BangBangTrajectory1D::getVelocity(double tt) const {
    auto trajTime = std::max(0.0f, (float)tt);
    
    if (trajTime >= getTotalTime()) {
        // requested time beyond final element
        return 0.0;
    }
    
    auto pieceIdx = findPartIdx(trajTime);
    auto piece = parts[pieceIdx];
    auto tPieceStart = pieceIdx < 1 ? 0 : parts[pieceIdx - 1].tEnd;
    auto t = trajTime - tPieceStart;
    return (double)(piece.v0 + (piece.acc * t));
}

double BangBangTrajectory1D::getAcceleration(double tt) const {
    float trajTime = std::max(0.0f, (float)tt);
    
    if (trajTime >= getTotalTime()) {
        // requested time beyond final element
        return 0.0;
    }
    
    return (double)findPart(trajTime).acc;
}

double BangBangTrajectory1D::getTotalTime() const {
    return parts[numParts - 1].tEnd;
}

std::unique_ptr<ITrajectory<double>> BangBangTrajectory1D::mirrored() const {
    auto mirrored = std::make_unique<BangBangTrajectory1D>();
    mirrored->numParts = numParts;
    for (int i = 0; i < numParts; i++) {
        mirrored->parts[i].tEnd = parts[i].tEnd;
        mirrored->parts[i].acc = -parts[i].acc;
        mirrored->parts[i].v0 = -parts[i].v0;
        mirrored->parts[i].s0 = -parts[i].s0;
    }
    return std::move(mirrored);
}

PosVelAcc<double> BangBangTrajectory1D::getValuesAtTime(double tt) const {
    float trajTime = std::max(0.0f, (float)tt);
    
    if (trajTime >= getTotalTime()) {
        // requested time beyond final element
        return PosVelAcc<double>(getPosition(tt), 0.0, 0.0);
    }
    
    auto pieceIdx = findPartIdx(trajTime);
    auto piece = parts[pieceIdx];
    auto tPieceStart = pieceIdx < 1 ? 0 : parts[pieceIdx - 1].tEnd;
    auto t = trajTime - tPieceStart;
    return PosVelAcc<double>(
        (double)(piece.s0 + (piece.v0 * t) + (0.5f * piece.acc * t * t)),
        (double)(piece.v0 + (piece.acc * t)),
        (double)piece.acc
    );
}

std::vector<double> BangBangTrajectory1D::getTimeSections() const {
    std::vector<double> sections;
    sections.reserve(numParts);
    for (int i = 0; i < numParts; i++) {
        sections.push_back((double)parts[i].tEnd);
    }
    return sections;
}

double BangBangTrajectory1D::getMaxSpeed() const {
    auto sections = getTimeSections();
    double maxSpeed = 0.0;
    for (auto time : sections) {
        maxSpeed = std::max(maxSpeed, std::abs(getVelocity(time)));
    }
    return maxSpeed;
}

int BangBangTrajectory1D::findPartIdx(double t) const {
    for (int i = 0; i < numParts; i++) {
        if (t < parts[i].tEnd) {
            return i;
        }
    }
    return numParts - 1;
}

BBTrajectoryPart BangBangTrajectory1D::findPart(double t) const {
    return parts[findPartIdx(t)];
}

BangBangTrajectory1D& BangBangTrajectory1D::generate(
    float initialPos, float finalPos, float initialVel, 
    float maxVel, float maxAcc) {
    
    float x0 = initialPos;
    float xd0 = initialVel;
    float xTrg = finalPos;
    float xdMax = maxVel;
    float xddMax = maxAcc;
    float sAtZeroAcc = velChangeToZero(x0, xd0, xddMax);

    if (sAtZeroAcc <= xTrg) {
        float sEnd = velTriToZero(x0, xd0, xdMax, xddMax);

        if (sEnd >= xTrg) {
            // Triangular profile
            calcTri(x0, xd0, xTrg, xddMax);
        } else {
            // Trapezoidal profile
            calcTrapz(x0, xd0, xdMax, xTrg, xddMax);
        }
    } else {
        // even with a full brake we miss xTrg
        float sEnd = velTriToZero(x0, xd0, -xdMax, xddMax);

        if (sEnd <= xTrg) {
            // Triangular profile
            calcTri(x0, xd0, xTrg, -xddMax);
        } else {
            // Trapezoidal profile
            calcTrapz(x0, xd0, -xdMax, xTrg, xddMax);
        }
    }
    return *this;
}

float BangBangTrajectory1D::velChangeToZero(float s0, float v0, float aMax) const {
    const float a = (0 >= v0) ? aMax : -aMax;
    const float t = -v0 / a;
    return s0 + (0.5f * v0 * t);
}

float BangBangTrajectory1D::velTriToZero(float s0, float v0, float v1, float aMax) const {
    const float a1 = (v1 >= v0) ? aMax : -aMax;
    const float a2 = (v1 >= v0) ? -aMax : aMax;

    const float t1 = (v1 - v0) / a1;
    const float s1 = s0 + (0.5f * (v0 + v1) * t1);

    const float t2 = -v1 / a2;
    return s1 + (0.5f * v1 * t2);
}

void BangBangTrajectory1D::calcTri(float s0, float v0, float s2, float a) {
    float sq;

    if (a > 0) {
        // + -
        sq = ((a * (s2 - s0)) + (0.5f * v0 * v0)) / (a * a);
    } else {
        // - +
        sq = ((-a * (s0 - s2)) + (0.5f * v0 * v0)) / (a * a);
    }

    const float t2 = (sq > 0.0f) ? std::sqrt(sq) : 0.0f;
    const float v1 = a * t2;
    const float t1 = (v1 - v0) / a;
    const float s1 = s0 + ((v0 + v1) * 0.5f * t1);

    parts[0].tEnd = t1;
    parts[0].acc = a;
    parts[0].v0 = v0;
    parts[0].s0 = s0;
    parts[1].tEnd = t1 + t2;
    parts[1].acc = -a;
    parts[1].v0 = v1;
    parts[1].s0 = s1;
    numParts = 2;
}

void BangBangTrajectory1D::calcTrapz(float s0, float v0, float v1, float s3, float aMax) {
    float a1 = (v0 > v1) ? -aMax : aMax;
    float a3 = (v1 > 0) ? -aMax : aMax;

    float t1 = (v1 - v0) / a1;
    float v2 = v1;
    float t3 = -v2 / a3;

    float s1 = s0 + (0.5f * (v0 + v1) * t1);
    float s2 = s3 - (0.5f * v2 * t3);
    float t2 = (s2 - s1) / v1;

    parts[0].tEnd = t1;
    parts[0].acc = a1;
    parts[0].v0 = v0;
    parts[0].s0 = s0;
    parts[1].tEnd = t1 + t2;
    parts[1].acc = 0;
    parts[1].v0 = v1;
    parts[1].s0 = s1;
    parts[2].tEnd = t1 + t2 + t3;
    parts[2].acc = a3;
    parts[2].v0 = v2;
    parts[2].s0 = s2;
    numParts = 3;
}

} // namespace ctrl