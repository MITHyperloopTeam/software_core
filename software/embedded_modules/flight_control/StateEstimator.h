#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <zcm/zcm.h>
#include "zcmgen_c/mithl_state_estimator_particle.h"
#include "zcmgen_c/mithl_state_estimator_particle_set.h"
#include "zcmgen_c/mithl_fiducial_t.h"
#include "zcmgen_c/mithl_state_estimator_config_t.h"
#include "arduino_imu_manager.h"

// Max particles we want to be allowed to track.
const int kMaxNumParticles = 100;
const double kPublishPeriod = 0.0333;
const double kConfigPublishPeriod = 0.5;
const double kDetailsPublishPeriod = 0.1;
const int kSDWindowSize = 5;
const int kTareSampleSize = 100;
const int kTareUpdatePeriod = 0.01;

/**
 * State Estimation Wrapper Class
 * Implements a multi-hypothesis state estimator.
 * Contains a couple of components:
 *  - The estimator, which can run in two modes:
 *    - Single-hypothesis EKF-guided integration of IMU signals
 *    - Multi-hypothesis dual-proposal PF with EKF particles
 *  - A stopping detector
 *
 * And supports a couple of mode switches:
 *  - Toggle stopping detector on and off
 *  - Toggle single vs multi hypothesis
 *  - Toggle usage of fiducial measurements
 */
class StateEstimator {
   public:
    // Initialize from given state
    StateEstimator(mithl_state_estimator_particle x0, int max_particles, IMUManager * front_nav_imu, IMUManager * rear_nav_imu, zcm_t * zcm);
    ~StateEstimator();

    // Reset to given state
    void reset(mithl_state_estimator_particle x0);
    
    // Publish info up to LCM
    void publish(double t);
    void publishDetails(double t);

    // Kicks off an update.
    // Call as often as possible, target couple hundred hz.
    // Returns true if update successful, false if some fault
    // has occured.
    bool update(double t);

    // Handles fiducial status messages
    void fiducial_t_handler(const mithl_fiducial_t * msg);

    // Handles config set
    void state_estimator_config_t_handler(const mithl_state_estimator_config_t * msg);

    // Tare IMU -- run repeatedly to tare more accurately.
    // k = weight put on old tare vs new tare. 1.0 = entirely new values.
    bool doIncrementalTare(double k);
    // start a 100-sample tare
    void startTare();

    bool get_stopped() { return stopped_; }
    void populate_mean_and_vars(float front_imu_mean[6], float rear_imu_mean[6], 
                                float front_imu_var[6], float rear_imu_var[6]){
        for (int i=0; i<6; i++){
            front_imu_mean[i] = imu_means_[i];
            rear_imu_mean[i] = imu_means_[i+6];
            front_imu_var[i] = imu_vars_[i];
            rear_imu_var[i] = imu_vars_[i+6];
        }
    }

  private:
    // zcm instance
    zcm_t * zcm_;

    // Tunable parameters
    mithl_state_estimator_config_t config_;

    // timers
    double last_publish_time_;
    double last_publish_details_time_;
    double last_publish_config_time_;
    double last_update_time_;
    double last_tare_update_time_;

    // health
    bool front_nav_imu_good_;
    bool rear_nav_imu_good_;

    // IMUs
    IMUManager * front_nav_imu_;
    double front_imu_x_bias_;
    double front_imu_x_bias_sum_;
    IMUManager * rear_nav_imu_;
    double rear_imu_x_bias_;
    double rear_imu_x_bias_sum_;
    int tare_counter_;

    bool fiducial_good_;
    double last_fiducial_time_;
    double last_time_since_last_strip_;
    double last_average_time_strip_;
    bool do_fiducial_update_;

    // Stopping detector
    bool stopped_;
    double last_not_stopping_time_;
    double last_stopping_detector_update_;
    double last_stopping_detector_mean_est_;
    double last_stopping_detector_max_var_est_;
    float imu_means_[12];
    float imu_vars_[12];
    float imu_data_circ_buffer_[kSDWindowSize][12];
    int stopping_detector_buf_ind_;


    // Particle legitimacy is kept track of in the id field -- negative = not legit
    int next_particle_id_;
    mithl_state_estimator_particle state_[kMaxNumParticles];

    // Used in particle generation and destruction step, where we spawn
    // 2 particles at a time.
    mithl_state_estimator_particle state_workspace_[kMaxNumParticles*2];
    
    bool updateTare(double t);

    bool updateStoppingDetector(double t);

    // Does a prediction step
    bool doProcessUpdate(double t, double dt);
    
    // Updates state with IMU data
    bool doAccelerationInnovationUpdate(double t, double dt);

    // Consumes a fiducial status message to do a fiducial update
    // This only adjusts state if a detection event is detected
    bool doFiducialMeasurementUpdate(double t, double dt);
    double calculateNearestStrip(double x);

};

#endif // STATE_ESTIMATOR_H
