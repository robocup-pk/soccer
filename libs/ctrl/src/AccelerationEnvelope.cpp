#include "AccelerationEnvelope.h"
#include <Python.h>
#include <iostream>
#include <fstream>
#include <sstream>

namespace ctrl {

AccelerationEnvelope::AccelerationEnvelope(const RobotParams& params) 
    : robot_params_(params), is_initialized_(false) {
    InitializePython();
    BuildLookupTable();
}

AccelerationEnvelope::~AccelerationEnvelope() {
    if (Py_IsInitialized()) {
        Py_Finalize();
    }
}

void AccelerationEnvelope::InitializePython() {
    if (!Py_IsInitialized()) {
        Py_Initialize();
        if (!Py_IsInitialized()) {
            std::cerr << "Failed to initialize Python interpreter" << std::endl;
            return;
        }
    }
    
    // Add project root to Python path so lookup_table.py can be imported
    PyRun_SimpleString("import sys, os, site");
    PyRun_SimpleString("site.main() if hasattr(site, 'main') else None");
    std::string repo_root = std::string(CMAKE_BUILD_DIR) + "/..";
    std::string cmd = std::string("sys.path.append('") + repo_root + "')";
    PyRun_SimpleString(cmd.c_str());
}

void AccelerationEnvelope::BuildLookupTable() {
    if (!Py_IsInitialized()) {
        std::cerr << "Python not initialized, cannot build lookup table" << std::endl;
        return;
    }
    
    try {
        // Create robot parameters file for Python
        CreateRobotParamsFile();
        
        // Import lookup_table module
        PyObject* pModule = PyImport_ImportModule("lookup_table");
        if (!pModule) {
            PyErr_Print();
            std::cerr << "Failed to import lookup_table module" << std::endl;
            return;
        }
        
        // Get RobotParams class
        PyObject* pRobotParamsClass = PyObject_GetAttrString(pModule, "RobotParams");
        if (!pRobotParamsClass) {
            PyErr_Print();
            Py_DECREF(pModule);
            return;
        }
        
        // Create robot parameters object
        PyObject* pParams = CreatePythonRobotParams(pRobotParamsClass);
        if (!pParams) {
            Py_DECREF(pRobotParamsClass);
            Py_DECREF(pModule);
            return;
        }
        
        // Get LookupTable class
        PyObject* pLookupTableClass = PyObject_GetAttrString(pModule, "LookupTable");
        if (!pLookupTableClass) {
            PyErr_Print();
            Py_DECREF(pParams);
            Py_DECREF(pRobotParamsClass);
            Py_DECREF(pModule);
            return;
        }
        
        // Build lookup table
        PyObject* pBuildMethod = PyObject_GetAttrString(pLookupTableClass, "build");
        if (!pBuildMethod) {
            PyErr_Print();
            Py_DECREF(pLookupTableClass);
            Py_DECREF(pParams);
            Py_DECREF(pRobotParamsClass);
            Py_DECREF(pModule);
            return;
        }
        
        // Call build method with parameters
        PyObject* pArgs = PyTuple_New(2);
        PyTuple_SetItem(pArgs, 0, pParams);  // pParams reference stolen
        PyTuple_SetItem(pArgs, 1, PyLong_FromLong(200000));  // num_samples
        
        PyObject* pLookupTable = PyObject_CallObject(pBuildMethod, pArgs);
        if (!pLookupTable) {
            PyErr_Print();
            std::cerr << "Failed to build lookup table" << std::endl;
            Py_DECREF(pArgs);
            Py_DECREF(pBuildMethod);
            Py_DECREF(pLookupTableClass);
            Py_DECREF(pRobotParamsClass);
            Py_DECREF(pModule);
            return;
        }
        
        // Extract lookup table data
        ExtractLookupTableData(pLookupTable);
        
        // Cleanup
        Py_DECREF(pLookupTable);
        Py_DECREF(pArgs);
        Py_DECREF(pBuildMethod);
        Py_DECREF(pLookupTableClass);
        Py_DECREF(pRobotParamsClass);
        Py_DECREF(pModule);
        
        is_initialized_ = true;
        std::cout << "Acceleration envelope initialized with " << lookup_entries_.size() 
                  << " sectors, alpha_max = " << alpha_max_ << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in BuildLookupTable: " << e.what() << std::endl;
    }
}

void AccelerationEnvelope::CreateRobotParamsFile() {
    // This could create a YAML file, but for simplicity we'll pass parameters directly
}

PyObject* AccelerationEnvelope::CreatePythonRobotParams(PyObject* pRobotParamsClass) {
    // Create constructor arguments
    PyObject* pArgs = PyTuple_New(0);  // Use default constructor
    PyObject* pKwargs = PyDict_New();
    
    // Set mass
    PyDict_SetItemString(pKwargs, "mass", PyFloat_FromDouble(robot_params_.mass_kg));
    
    // Set inertia_z
    PyDict_SetItemString(pKwargs, "inertia_z", PyFloat_FromDouble(robot_params_.inertia_z));
    
    // Set friction coefficient
    PyDict_SetItemString(pKwargs, "mu", PyFloat_FromDouble(robot_params_.friction_coeff));
    
    // Set wheel_force_max
    PyDict_SetItemString(pKwargs, "wheel_force_max", PyFloat_FromDouble(robot_params_.wheel_force_max));
    
    // Create wheel_positions list
    PyObject* pPositions = PyList_New(robot_params_.wheel_positions_m.size());
    for (size_t i = 0; i < robot_params_.wheel_positions_m.size(); ++i) {
        PyObject* pPos = PyTuple_New(2);
        PyTuple_SetItem(pPos, 0, PyFloat_FromDouble(robot_params_.wheel_positions_m[i].first));
        PyTuple_SetItem(pPos, 1, PyFloat_FromDouble(robot_params_.wheel_positions_m[i].second));
        PyList_SetItem(pPositions, i, pPos);
    }
    PyDict_SetItemString(pKwargs, "wheel_positions", pPositions);
    
    // Create wheel_dirs list (unit vectors from angles)
    PyObject* pDirs = PyList_New(robot_params_.wheel_angles_rad.size());
    for (size_t i = 0; i < robot_params_.wheel_angles_rad.size(); ++i) {
        double angle = robot_params_.wheel_angles_rad[i];
        PyObject* pDir = PyTuple_New(2);
        PyTuple_SetItem(pDir, 0, PyFloat_FromDouble(std::cos(angle)));
        PyTuple_SetItem(pDir, 1, PyFloat_FromDouble(std::sin(angle)));
        PyList_SetItem(pDirs, i, pDir);
    }
    PyDict_SetItemString(pKwargs, "wheel_dirs", pDirs);
    
    // Call constructor
    PyObject* pParams = PyObject_Call(pRobotParamsClass, pArgs, pKwargs);
    
    Py_DECREF(pArgs);
    Py_DECREF(pKwargs);
    
    return pParams;
}

void AccelerationEnvelope::ExtractLookupTableData(PyObject* pLookupTable) {
    // Get entries
    PyObject* pEntries = PyObject_GetAttrString(pLookupTable, "_entries");
    if (!pEntries) {
        PyErr_Print();
        return;
    }
    
    // Get alpha_max
    PyObject* pAlphaMax = PyObject_GetAttrString(pLookupTable, "_alpha_max");
    if (!pAlphaMax) {
        PyErr_Print();
        Py_DECREF(pEntries);
        return;
    }
    
    alpha_max_ = PyFloat_AsDouble(pAlphaMax);
    
    // Extract entries
    Py_ssize_t size = PyList_Size(pEntries);
    lookup_entries_.clear();
    lookup_entries_.reserve(size);
    
    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* pEntry = PyList_GetItem(pEntries, i);
        
        PyObject* pAngle = PyObject_GetAttrString(pEntry, "angle_deg");
        PyObject* pAMax = PyObject_GetAttrString(pEntry, "a_max");
        
        if (pAngle && pAMax) {
            LookupEntry entry;
            entry.angle_deg = PyFloat_AsDouble(pAngle);
            entry.a_max = PyFloat_AsDouble(pAMax);
            lookup_entries_.push_back(entry);
        }
        
        Py_XDECREF(pAngle);
        Py_XDECREF(pAMax);
    }
    
    Py_DECREF(pEntries);
    Py_DECREF(pAlphaMax);
}

std::pair<double, double> AccelerationEnvelope::QueryAcceleration(double direction_rad) const {
    if (!is_initialized_ || lookup_entries_.empty()) {
        // Fallback to conservative values
        return std::make_pair(2.0, 10.0);
    }
    
    // Convert to degrees [0, 360)
    double deg = std::fmod(direction_rad * 180.0 / M_PI + 360.0, 360.0);
    
    // Find adjacent sectors for interpolation
    size_t n = lookup_entries_.size();
    size_t idx = 0;
    
    for (size_t i = 0; i < n; ++i) {
        if (lookup_entries_[i].angle_deg <= deg) {
            idx = i;
        } else {
            break;
        }
    }
    
    size_t idx_next = (idx + 1) % n;
    
    const LookupEntry& entry1 = lookup_entries_[idx];
    const LookupEntry& entry2 = lookup_entries_[idx_next];
    
    // Handle wrap-around at 360°/0°
    double angle1 = entry1.angle_deg;
    double angle2 = entry2.angle_deg;
    if (angle2 < angle1) {
        angle2 += 360.0;
    }
    if (deg < angle1) {
        deg += 360.0;
    }
    
    // Linear interpolation
    double t = (deg - angle1) / (angle2 - angle1);
    double a_max_interp = entry1.a_max * (1.0 - t) + entry2.a_max * t;
    
    return std::make_pair(a_max_interp, alpha_max_);
}

Eigen::Vector3d AccelerationEnvelope::ComputeMaxAcceleration(double direction_rad) const {
    auto [a_max, alpha_max] = QueryAcceleration(direction_rad);
    
    return Eigen::Vector3d(
        a_max * std::cos(direction_rad),
        a_max * std::sin(direction_rad),
        alpha_max
    );
}

bool AccelerationEnvelope::IsAccelerationFeasible(const Eigen::Vector3d& acceleration) const {
    if (!is_initialized_) {
        return false;
    }
    
    double direction = std::atan2(acceleration[1], acceleration[0]);
    double magnitude = std::sqrt(acceleration[0]*acceleration[0] + acceleration[1]*acceleration[1]);
    
    auto [a_max, alpha_max] = QueryAcceleration(direction);
    
    return magnitude <= a_max && std::abs(acceleration[2]) <= alpha_max;
}

void AccelerationEnvelope::PrintEnvelopeInfo() const {
    if (!is_initialized_) {
        std::cout << "Acceleration envelope not initialized" << std::endl;
        return;
    }
    
    std::cout << "=== Acceleration Envelope Info ===" << std::endl;
    std::cout << "Number of sectors: " << lookup_entries_.size() << std::endl;
    std::cout << "Alpha max (angular): " << alpha_max_ << " rad/s²" << std::endl;
    
    double min_a = std::numeric_limits<double>::max();
    double max_a = std::numeric_limits<double>::min();
    
    for (const auto& entry : lookup_entries_) {
        min_a = std::min(min_a, entry.a_max);
        max_a = std::max(max_a, entry.a_max);
    }
    
    std::cout << "Linear acceleration range: [" << min_a << ", " << max_a << "] m/s²" << std::endl;
    
    // Sample a few directions
    std::cout << "\nSample accelerations:" << std::endl;
    for (double angle_deg = 0; angle_deg < 360; angle_deg += 45) {
        double angle_rad = angle_deg * M_PI / 180.0;
        auto [a_max, alpha_max] = QueryAcceleration(angle_rad);
        std::cout << "  " << std::setw(3) << (int)angle_deg << "°: a_max = " 
                  << std::fixed << std::setprecision(2) << a_max << " m/s²" << std::endl;
    }
}

} // namespace ctrl