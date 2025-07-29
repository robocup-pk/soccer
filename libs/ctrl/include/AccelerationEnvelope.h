#ifndef ACCELERATION_ENVELOPE_H
#define ACCELERATION_ENVELOPE_H

#include <Python.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <iomanip>

namespace ctrl {

struct RobotParams {
    double mass_kg = 2.3;           // Robot mass in kg
    double inertia_z = 0.0085;      // Rotational inertia about z-axis (kg⋅m²)
    double friction_coeff = 0.8;    // Coefficient of friction
    double wheel_force_max = 1.5;   // Maximum wheel force in Newtons
    
    // Custom wheel configuration (angles in radians from robot x-axis)
    std::vector<double> wheel_angles_rad = {
        -30 * M_PI / 180,   // wheel 1: -30°
        45 * M_PI / 180,    // wheel 2: +45° 
        135 * M_PI / 180,   // wheel 3: +135°
        -150 * M_PI / 180   // wheel 4: -150°
    };
    
    // Wheel positions from robot center (m)
    std::vector<std::pair<double, double>> wheel_positions_m = {
        {0.045601, 0.080113},   // wheel 1
        {-0.064798, 0.065573},  // wheel 2
        {-0.064798, -0.065573}, // wheel 3
        {0.045601, -0.080113}   // wheel 4
    };
};

struct LookupEntry {
    double angle_deg;  // Centre angle of the sector
    double a_max;      // Safe linear acceleration magnitude (m/s²)
};

/**
 * @brief Acceleration envelope using Monte Carlo lookup table
 * 
 * This class interfaces with the Python lookup_table.py to compute
 * the vehicle's acceleration envelope considering:
 * - Vehicle dynamics and weight transfer
 * - Friction constraints
 * - Custom wheel configuration
 * - Real-time acceleration queries
 */
class AccelerationEnvelope {
public:
    explicit AccelerationEnvelope(const RobotParams& params);
    ~AccelerationEnvelope();
    
    /**
     * @brief Query maximum safe acceleration in given direction
     * @param direction_rad Direction angle in radians
     * @return (a_max, alpha_max) - linear and angular acceleration limits
     */
    std::pair<double, double> QueryAcceleration(double direction_rad) const;
    
    /**
     * @brief Compute maximum acceleration vector for given direction
     * @param direction_rad Direction angle in radians  
     * @return [ax_max, ay_max, alpha_max]
     */
    Eigen::Vector3d ComputeMaxAcceleration(double direction_rad) const;
    
    /**
     * @brief Check if given acceleration is feasible
     * @param acceleration [ax, ay, alpha] acceleration vector
     * @return true if acceleration is within envelope
     */
    bool IsAccelerationFeasible(const Eigen::Vector3d& acceleration) const;
    
    /**
     * @brief Get maximum angular acceleration
     */
    double GetMaxAngularAcceleration() const { return alpha_max_; }
    
    /**
     * @brief Check if envelope is initialized
     */
    bool IsInitialized() const { return is_initialized_; }
    
    /**
     * @brief Print envelope information for debugging
     */
    void PrintEnvelopeInfo() const;

private:
    RobotParams robot_params_;
    std::vector<LookupEntry> lookup_entries_;
    double alpha_max_;
    bool is_initialized_;
    
    // Python integration
    void InitializePython();
    void BuildLookupTable();
    void CreateRobotParamsFile();
    PyObject* CreatePythonRobotParams(PyObject* pRobotParamsClass);
    void ExtractLookupTableData(PyObject* pLookupTable);
};

} // namespace ctrl

#endif // ACCELERATION_ENVELOPE_H