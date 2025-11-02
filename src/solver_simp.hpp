#pragma once

#include "rtklib.h"
#include <string>

/**
 * Simple Single Point Positioning (SPP) Solver
 * Uses RTKLIB's pntpos function
 */
class SimpleSolver {
public:
    SimpleSolver();
    ~SimpleSolver();
    
    /**
     * Compute position from observations and navigation data
     * @param obs Observations from parser
     * @param nav Navigation data from parser
     * @return true if solution computed successfully
     */
    bool computePosition(const obs_t& obs, const nav_t& nav);
    
    /**
     * Get the latest solution
     */
    const sol_t& getSolution() const { return solution; }
    
    /**
     * Get solution statistics
     */
    struct Statistics {
        unsigned long totalAttempts;     // Total position attempts
        unsigned long successfulSolutions; // Successful solutions
        unsigned long failedSolutions;   // Failed solutions
        
        Statistics() : totalAttempts(0), successfulSolutions(0), failedSolutions(0) {}
    };
    
    const Statistics& getStats() const { return stats; }
    void resetStats() { stats = Statistics(); }
    
    /**
     * Check if last solution is valid
     */
    bool hasValidSolution() const;
    
    /**
     * Get position in geodetic coordinates (lat, lon, height)
     */
    void getGeodeticPosition(double& lat, double& lon, double& height) const;
    
    /**
     * Get position accuracy (horizontal and vertical)
     */
    void getAccuracy(double& hAcc, double& vAcc) const;

private:
    prcopt_t options;      // Processing options
    sol_t solution;        // Current solution
    double azel[MAXOBS*2]; // Azimuth/elevation angles
    ssat_t ssat[MAXSAT];   // Satellite status
    char msg[256];         // Error/warning message
    
    Statistics stats;
    
    void initOptions();
};