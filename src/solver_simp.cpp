#include "solver_simp.hpp"
#include <cstring>
#include <cmath>

// RTKLIB external function
extern "C" {
    // Main positioning function from pntpos.c
    extern int pntpos(const obsd_t *obs, int n, const nav_t *nav,
                      const prcopt_t *opt, sol_t *sol, double *azel,
                      ssat_t *ssat, char *msg);
}

SimpleSolver::SimpleSolver() {
    memset(&solution, 0, sizeof(sol_t));
    memset(azel, 0, sizeof(azel));
    memset(ssat, 0, sizeof(ssat));
    memset(msg, 0, sizeof(msg));
    
    initOptions();
}

SimpleSolver::~SimpleSolver() {
    // Nothing to clean up
}

void SimpleSolver::initOptions() {
    // Initialize with default options
    memset(&options, 0, sizeof(prcopt_t));
    
    // Set Single Point Positioning mode
    options.mode = PMODE_SINGLE;  // Single point positioning
    
    // Number of frequencies (1 for L1 only)
    options.nf = 1;
    
    // Elevation mask (degrees) - reject satellites below this angle
    options.elmin = 15.0 * D2R;  // 15 degrees
    
    // Satellite systems to use
    options.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP | SYS_QZS;
    
    // Ionosphere correction: broadcast model
    options.ionoopt = IONOOPT_BRDC;  // Use broadcast ionosphere model
    
    // Troposphere correction: Saastamoinen model
    options.tropopt = TROPOPT_SAAS;  // Saastamoinen model
    
    // Satellite ephemeris: broadcast
    options.sateph = EPHOPT_BRDC;
    
    // Positioning mode
    options.modear = ARMODE_OFF;  // No ambiguity resolution (we're doing SPP, not RTK)
    
    // Code smoothing
    options.intpref = 0;  // No interpolation reference
    
    // Dynamics model
    options.dynamics = 0;  // Static positioning
    
    // Earth tide correction
    options.tidecorr = 0;  // No tide correction for simplicity
    
    // Max iteration for position computation
    options.maxiter = 10;
    
    // Standard deviation for position
    // These are just initial values, pntpos will compute actual accuracy
    options.sclkstab = 5E-12;  // Satellite clock stability
    options.thresar[0] = 3.0;  // AR validation threshold
}

bool SimpleSolver::computePosition(const obs_t& obs, const nav_t& nav) {
    stats.totalAttempts++;
    
    // Check if we have observations
    if (obs.n == 0) {
        stats.failedSolutions++;
        return false;
    }
    
    // Clear previous solution
    memset(&solution, 0, sizeof(sol_t));
    memset(azel, 0, sizeof(azel));
    memset(ssat, 0, sizeof(ssat));
    memset(msg, 0, sizeof(msg));
    
    // Call RTKLIB pntpos function
    // Returns 1 if position computed successfully
    int result = pntpos(obs.data,        // Observation data
                       obs.n,            // Number of observations
                       &nav,             // Navigation data (ephemeris)
                       &options,         // Processing options
                       &solution,        // Output: solution
                       azel,             // Output: azimuth/elevation
                       ssat,             // Output: satellite status
                       msg);             // Output: error message
    
    if (result) {
        stats.successfulSolutions++;
        return true;
    } else {
        stats.failedSolutions++;
        return false;
    }
}

bool SimpleSolver::hasValidSolution() const {
    // Check if solution status indicates a valid position
    // sol.stat values:
    // 0: no solution
    // 1: single point positioning (SPP)
    // 2-5: various RTK modes (not used in SPP)
    return solution.stat >= SOLQ_SINGLE;
}

void SimpleSolver::getGeodeticPosition(double& lat, double& lon, double& height) const {
    if (!hasValidSolution()) {
        lat = lon = height = 0.0;
        return;
    }
    
    // Solution is in ECEF (Earth-Centered Earth-Fixed) coordinates
    // Convert to geodetic (lat, lon, height)
    double pos[3];
    ecef2pos(solution.rr, pos);  // RTKLIB function to convert ECEF to lat/lon/height
    
    lat = pos[0] * R2D;    // Convert radians to degrees
    lon = pos[1] * R2D;    // Convert radians to degrees
    height = pos[2];       // Height in meters
}

void SimpleSolver::getAccuracy(double& hAcc, double& vAcc) const {
    if (!hasValidSolution()) {
        hAcc = vAcc = 0.0;
        return;
    }
    
    // solution.qr contains position covariance
    // qr[0] = variance of X (ECEF)
    // qr[1] = variance of Y (ECEF)
    // qr[2] = variance of Z (ECEF)
    
    // For horizontal accuracy, use X and Y components
    // For vertical accuracy, use Z component
    
    // Standard deviation = sqrt(variance)
    double sdX = sqrt(solution.qr[0]);
    double sdY = sqrt(solution.qr[1]);
    double sdZ = sqrt(solution.qr[2]);
    
    // Horizontal accuracy (2D RMS)
    hAcc = sqrt(sdX * sdX + sdY * sdY);
    
    // Vertical accuracy
    vAcc = sdZ;
}