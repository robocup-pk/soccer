#pragma once

// Corresponds to EPlanarCurveSegmentType.java

/**
 * @brief Defines different types of planar (2D) curve segments.
 */
enum class EPlanarCurveSegmentType {
    /** Position only */
    POINT,
    /** Position and velocity */
    FIRST_ORDER,
    /** Position, velocity, and acceleration */
    SECOND_ORDER
};
