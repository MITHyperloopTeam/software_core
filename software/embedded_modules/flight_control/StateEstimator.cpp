#include "StateEstimator.h"
// comms
#include "arduino_time_manager.h"
#include "arduino_matrix_math.h"
#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_state_estimator_state_t.h"

double calc_pod_acceleration(mithl_state_estimator_particle){
  // TODO(gizatt) implement dynamics model
  return 0.0;
}

void fd_o_handler_helper(const zcm_recv_buf_t * rbuf, const char * channel, 
                   const mithl_fiducial_t * msg, void * user){
  ((StateEstimator*)user)->fiducial_t_handler(msg);
}

void config_handler_helper(const zcm_recv_buf_t * rbuf, const char * channel, 
                   const mithl_state_estimator_config_t * msg, void * user){
  ((StateEstimator*)user)->state_estimator_config_t_handler(msg);
}

void reset_handler_helper(const zcm_recv_buf_t * rbuf, const char * channel, 
                   const mithl_trigger_t * msg, void * user){

  mithl_state_estimator_particle x0;
  x0.id = 0;
  x0.weight = 1.0;
  for (int i=0; i<3; i++){
    x0.mu[i] = 0.0;
    for (int j=0; j<3; j++){
      x0.Sigma[i][j] = 0.0;
    }
  }
  x0.Sigma[0][0] = 0.0;
  x0.Sigma[1][1] = 0.0;
  x0.Sigma[2][2] = 0.0;
  ((StateEstimator*)user)->reset(x0);
}

void tare_handler_helper(const zcm_recv_buf_t * rbuf, const char * channel, 
                   const mithl_trigger_t * msg, void * user){
  ((StateEstimator*)user)->startTare();
}


StateEstimator::StateEstimator(mithl_state_estimator_particle x0, 
                               int max_particles, 
                               IMUManager * front_nav_imu, 
                               IMUManager * rear_nav_imu, 
                               zcm_t * zcm_) :
  zcm_(zcm_),
  next_particle_id_(0),
  front_nav_imu_(front_nav_imu),
  rear_nav_imu_(rear_nav_imu),
  rear_imu_x_bias_(0.0),
  front_imu_x_bias_(0.0),
  last_stopping_detector_mean_est_(0.0),
  last_stopping_detector_max_var_est_(0.0),
  last_average_time_strip_(0.0),
  tare_counter_(0)
  {
  // Load in default params
  config_.Sigma_meas_acc = 0.01;
  config_.Sigma_process_add_pos = 0.01;
  config_.Sigma_process_add_vel = 0.01;
  config_.Sigma_process_add_acc = 20.0;
  config_.fiducial_timeout = 1.0;
  config_.fiducial_FP_rate = 0.0005;
  config_.fiducial_sigma = 5.0; // meters, uncertainty per strip detection
  config_.fiducial_vel_sigma = 0.02; // meters, uncertainty per strip detection, multiplied by velocity estimate
  config_.fiducial_max_vel_difference = 20; // meters, uncertainty per strip detection
  config_.incremental_tare_alpha = 0.9995;
  config_.use_fiducial_detections = true;
  config_.use_multiple_particles = false;
  config_.max_particles = max_particles;
    // Tube config -- to be moved to an internal config management by message
  config_.tube_length = 1280.16;
  config_.fiducial_separation = 30.48;
  config_.distance_after_last_fiducial = 30.48;
  config_.fiducial_width = 0.0981;
    // Stoppint detector
  config_.SD_update_period = 0.01;
  config_.SD_stopping_threshold = 0.005; // variance threshold across all channels.
  config_.SD_forward_acc_threshold = 0.2;
  config_.SD_stop_timeout = 0.5;

  last_fiducial_time_ = get_current_time();
  last_publish_time_ = get_current_time();
  last_update_time_ = get_current_time();
  last_publish_details_time_ = get_current_time();
  last_publish_config_time_ = get_current_time();
  last_not_stopping_time_ = get_current_time();
  last_tare_update_time_ = get_current_time();

  rear_imu_x_bias_ = 0.0;
  front_imu_x_bias_ = 0.0;

  reset(x0);
  
  mithl_fiducial_t_subscribe(zcm_, "_FD_O", &fd_o_handler_helper, this);
  mithl_trigger_t_subscribe(zcm_, "RESET", &reset_handler_helper, this);
  mithl_state_estimator_config_t_subscribe(zcm_, "_FC_SE_CONFIG_SET", &config_handler_helper, this);
  mithl_trigger_t_subscribe(zcm_, "TARE", &tare_handler_helper, this);
}

StateEstimator::~StateEstimator() {
}

void StateEstimator::reset(mithl_state_estimator_particle x0) {
  last_time_since_last_strip_ = -1.0;

  stopped_ = true;
  last_stopping_detector_update_ = get_current_time();
  stopping_detector_buf_ind_ = 0;
  for (int i=0; i<kSDWindowSize; i++){
    for (int j=0; j<12; j++){
      imu_data_circ_buffer_[i][j] = 0.0;
    }
  }
  for (int i=0; i<12; i++){
    imu_means_[i] = 0.0;
    imu_vars_[i] = 0.0;
  }

  front_nav_imu_good_ = false;
  rear_nav_imu_good_ = false;
  fiducial_good_ = false;
  do_fiducial_update_ = false;

  // set us up with exactly one particle
  for (int i=0; i<kMaxNumParticles; i++){
    state_[i].id = -1;
    state_[i].weight = 0.0;
  }
  state_[0] = x0;


  // take the user input id number and start counting from there
  next_particle_id_ = state_[0].id + 1;
}

bool StateEstimator::update(double t) {
  double dt = t - last_update_time_;
  last_update_time_ = t;

  bool status = true;

  status = updateTare(t);


  if (status) {
    status = updateStoppingDetector(t);
  }

  if (status){
    status = doProcessUpdate(t, dt);
  }

  if (status){
    status = doAccelerationInnovationUpdate(t, dt);
  }
  if (status && config_.use_fiducial_detections){
    if (t - last_fiducial_time_ < config_.fiducial_timeout){
      fiducial_good_ = true;
      if (do_fiducial_update_){
        status = doFiducialMeasurementUpdate(t, dt);
      }
    } else {
      fiducial_good_ = false;
      status = false;
    }
  }
  
  if (t - last_publish_time_ > kPublishPeriod || t < last_publish_time_ - 1.0){
    last_publish_time_ = t;
    publish(t);
  }

  if (t - last_publish_details_time_ > kDetailsPublishPeriod || t < last_publish_details_time_ - 1.0){
    last_publish_details_time_ = t;
    publishDetails(t);
  }

  if (t - last_publish_config_time_ > kConfigPublishPeriod || t < last_publish_config_time_ - 1.0){
    last_publish_config_time_= t;
    mithl_state_estimator_config_t_publish(zcm_, "_FC_SE_CONFIG", &config_);
  }

  return status;
}

bool StateEstimator::updateTare(double t){
  if (tare_counter_ > 0 && t - last_tare_update_time_ > kTareUpdatePeriod){
    last_tare_update_time_ = t;
    tare_counter_ -= 1;
    front_nav_imu_good_ = front_nav_imu_->is_imu_healthy();
    rear_nav_imu_good_ = rear_nav_imu_->is_imu_healthy();
    if (!front_nav_imu_good_ and !rear_nav_imu_good_){
      return false;
    }
    if (front_nav_imu_good_){
      front_imu_x_bias_sum_ += front_nav_imu_->imu_data_[0];
    }
    if (rear_nav_imu_good_){
      rear_imu_x_bias_sum_ += rear_nav_imu_->imu_data_[0];
    }

    if (tare_counter_== 0){
      front_imu_x_bias_ = front_imu_x_bias_sum_ / (float) kTareSampleSize;
      rear_imu_x_bias_ = rear_imu_x_bias_sum_ / (float) kTareSampleSize;
    }
  }
  return true;
}
bool StateEstimator::updateStoppingDetector(double t){
  if (t - last_stopping_detector_update_ > config_.SD_update_period || t < last_stopping_detector_update_ - 1.0){
    last_stopping_detector_update_ = t;

    // floats in this pinfunction because the IMU is only that precise

    float old_data[12];
    memcpy(old_data, imu_data_circ_buffer_[stopping_detector_buf_ind_], sizeof(float)*12);

    float new_data[12];
    for (int i=0; i<12; i++) new_data[i] = 0.0;

    front_nav_imu_good_ = front_nav_imu_->is_imu_healthy();
    rear_nav_imu_good_ = rear_nav_imu_->is_imu_healthy();
    if (front_nav_imu_good_){
      memcpy(new_data, front_nav_imu_->imu_data_, sizeof(float)*6);
    }
    if (rear_nav_imu_good_){
      memcpy(new_data + 6, rear_nav_imu_->imu_data_, sizeof(float)*6);
    }
    if (!front_nav_imu_good_&& !rear_nav_imu_good_)
      return false;

    memcpy(imu_data_circ_buffer_[stopping_detector_buf_ind_], new_data, sizeof(float)*12);
    stopping_detector_buf_ind_ = (stopping_detector_buf_ind_ + 1) % kSDWindowSize;

    float new_mean[12];
    for (int i=0; i<12; i++){
      new_mean[i] = 0.0;
      for (int j=0; j<kSDWindowSize; j++){
        new_mean[i] += imu_data_circ_buffer_[j][i] / (float)kSDWindowSize;
      }
      imu_vars_[i] += (new_data[i] - old_data[i])*(new_data[i] - new_mean[i] + old_data[i] - imu_means_[i]) / ((float)kSDWindowSize - 1);
    }

    memcpy(imu_means_, new_mean, sizeof(float)*12);

    // Finally, actual stopping test:
    float true_mean = 0.0;
    int divisor = 0;
    if (front_nav_imu_good_){
      true_mean += imu_means_[0] - front_imu_x_bias_;
      divisor ++;
    }
    if (rear_nav_imu_good_){
      true_mean += -imu_means_[6] + rear_imu_x_bias_;
      divisor ++;
    }
    true_mean /= (float) divisor;

    bool stopped = true;
    float max_var = 0.0;
    for (int i=0; i<12; i++){
      if (fabs(imu_vars_[i]) > max_var){
        max_var = fabs(imu_vars_[i]);
      }
    }

    if (true_mean > config_.SD_forward_acc_threshold){
      stopped = false;
    } else if (max_var >= config_.SD_stopping_threshold) {
        stopped = false;
    }

    if (!stopped){
      last_not_stopping_time_ = t;
    }
    if (t - last_not_stopping_time_ > config_.SD_stop_timeout){
      stopped = true;
    } else {
      stopped = false;
    }
    stopped_ = stopped;
    last_stopping_detector_mean_est_ = true_mean;
    last_stopping_detector_max_var_est_ = max_var;
  }
  return true;
}
void printMatrix(char * str, XenMatrix::Mat3<float> m){
  printf("%s:", str);
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      printf("%f, ", m(i, j));
    }
  }
  printf("\n");
}
bool StateEstimator::doProcessUpdate(double t, double dt){
  for (int i=0; i<config_.max_particles; i++){
    if (state_[i].id >= 0 && state_[i].weight > 0){
      if (stopped_){
        state_[i].mu[0] = state_[i].mu[0];
        state_[i].mu[1] = 0.0; 
        state_[i].mu[2] = 0.0;
      } else {
        // Process model is double integration from acc down to velocity
        // and update of expected acceleration based on pod model
        state_[i].mu[0] += dt * state_[i].mu[1];
        state_[i].mu[1] += dt * state_[i].mu[2];
        state_[i].mu[2] = 0.0; // Predicted acceleration, for now assume no motion
      }

      // Jacobian of the process update
      XenMatrix::Mat3<float> Jac;
      Jac(0, 0) = 1.0;
      Jac(0, 1) = -dt;
      Jac(1, 1) = 1.0;
      Jac(1, 2) = -dt;
      Jac(2, 2) = 0.0;

      XenMatrix::Mat3<float> Sigma_additive;
      if (!stopped_){
        Sigma_additive(0, 0) = config_.Sigma_process_add_pos*dt; // EKF seems to be underestimating error... patching it here for now    
        Sigma_additive(1, 1) = config_.Sigma_process_add_vel*dt;    
        Sigma_additive(2, 2) = config_.Sigma_process_add_acc; // Continuously uncertain of acceleration 
      }

      XenMatrix::Mat3<float> Sigma_new = Jac * XenMatrix::Mat3<float>(state_[i].Sigma) * Jac.transpose() + Sigma_additive;

      for (int k=0; k<3; k++){
        for (int l=0; l<3; l++){
          state_[i].Sigma[k][l] = Sigma_new(k, l);
        }
      }

    }
  }
  return true;
}

bool StateEstimator::doIncrementalTare(double k){
  front_nav_imu_good_ = front_nav_imu_->is_imu_healthy();
  rear_nav_imu_good_ = rear_nav_imu_->is_imu_healthy();
  if (front_nav_imu_good_){
    double x = front_nav_imu_->imu_data_[0];
    front_imu_x_bias_ = front_imu_x_bias_*(k) + x*(1.0-k);
  }
  if (rear_nav_imu_good_){
    double x = rear_nav_imu_->imu_data_[0];
    rear_imu_x_bias_ = rear_imu_x_bias_*(k) + x*(1.0-k);
  }
  if (!front_nav_imu_good_ && !rear_nav_imu_good_){
    return false;
  } else {
    return true;
  }
}

void StateEstimator::startTare(){
  tare_counter_ = kTareSampleSize;
  rear_imu_x_bias_sum_ = 0.0;
  front_imu_x_bias_sum_ = 0.0;
}

bool StateEstimator::doAccelerationInnovationUpdate(double t, double dt){
  // IMU sanity checking and taring
  double avg_imu_x = 0.0;
  front_nav_imu_good_ = front_nav_imu_->is_imu_healthy();
  rear_nav_imu_good_ = rear_nav_imu_->is_imu_healthy();
  if (front_nav_imu_good_){
    double x = front_nav_imu_->imu_data_[0];
    avg_imu_x += (x - front_imu_x_bias_);
  }
  if (rear_nav_imu_good_){
    double x = rear_nav_imu_->imu_data_[0];
    avg_imu_x += -(x - rear_imu_x_bias_);
  }
  if (front_nav_imu_good_ || rear_nav_imu_good_){
    avg_imu_x /= (double) front_nav_imu_good_ + (double) rear_nav_imu_good_;
  } else {
    return false;
  }

  // If we've been stopped for long enough, re-tare IMU

  for (int i=0; i<config_.max_particles; i++){
    if (state_[i].id >= 0 && state_[i].weight > 0){
      if (stopped_){
        //doIncrementalTare(config_.incremental_tare_alpha);
        // Zero velocity, zero acceleration, zero velocity and acceleration uncertainty,
        // but hold onto position uncertainty
        state_[i].mu[1] = 0.0;
        state_[i].mu[2] = avg_imu_x;
        state_[i].Sigma[0][1] = 0.0;
        state_[i].Sigma[0][2] = 0.0;
        state_[i].Sigma[1][0] = 0.0;
        state_[i].Sigma[1][1] = 0.0;
        state_[i].Sigma[1][2] = 0.0;
        state_[i].Sigma[2][0] = 0.0;
        state_[i].Sigma[2][1] = 0.0;
        state_[i].Sigma[2][2] = 0.0;
      } else {
        // Innovation = 
        double innovation = avg_imu_x - state_[i].mu[2];
        // H = [0, 0, 1]
        XenMatrix::Vec3<float> H(0., 0., 1.);
        XenMatrix::Mat3<float> Sigma(state_[i].Sigma);
        // S = H Sigma H.' + a little bit of noise, calculating out from H above...
        double S = H * Sigma * H + config_.Sigma_meas_acc;

        // K = Sigma * H.' * S^(-1) = rightmost column of Sigma * S^(-1)
        XenMatrix::Vec3<float> K = Sigma * H / (float) S;
        // x_new = x + K * innovation
        for (int k=0; k<3; k++){
          state_[i].mu[k] += K(k)*innovation;
        }
        // Sigma_new = (I - K*H)*Sigma
        // (I - K*H) -> []
        XenMatrix::Mat3<float> eye;
        eye(0, 0) = 1.0; eye(1, 1) = 1.0; eye(2, 2) = 1.0;
        XenMatrix::Mat3<float> Sigma_new = (eye - K.outer(H)) * Sigma;
        for (int k=0; k<3; k++){
          for (int l=0; l<3; l++){
            state_[i].Sigma[k][l] = Sigma_new(k, l);
          }
        }
      }
    }
  }
  return true;
}

double StateEstimator::calculateNearestStrip(double x){
  double distance_to_first_strip = fmod(config_.tube_length, config_.fiducial_separation);
  if (x < distance_to_first_strip) {
    return distance_to_first_strip;
  } else if (x > config_.tube_length) {
    return config_.tube_length;
  } else {
    double dx_forward = ceil((x - distance_to_first_strip) / config_.fiducial_separation) * config_.fiducial_separation - (x - distance_to_first_strip);
    double dx_backward = (x - distance_to_first_strip) - floor((x - distance_to_first_strip) / config_.fiducial_separation) * config_.fiducial_separation;
    if (dx_backward <= dx_forward)
      return x - dx_backward;
    else
      return x + dx_forward;
  }
}
bool StateEstimator::doFiducialMeasurementUpdate(double t, double dt){
  if (config_.use_multiple_particles){
    printf("NOT IMPLEMENTED\n");
    for (int i=0; i<config_.max_particles; i++){
    }
  } else {
    double x = state_[0].mu[0];
    double xd = state_[0].mu[1];
    double sigma = state_[0].Sigma[0][0];
    double sigmaVel = state_[0].Sigma[1][1];
    double sigma2 = powf(sigma, 2);
    double sigmaVel2 = powf(sigmaVel, 2);
    double fiducialSigma2 = powf(config_.fiducial_sigma, 2);
    double fiducialSigmaVel2 = powf(config_.fiducial_vel_sigma*state_[0].mu[1], 2);
    double projected_travel = last_time_since_last_strip_*state_[0].mu[1];
    double estimated_speed = config_.fiducial_width / last_average_time_strip_;
    double nearest_strip = calculateNearestStrip(x - projected_travel) + projected_travel;
    // Weigh the probability of the movement (which is the product of the distributions
    // p(x) and p(strip detection), against a false positive probability
    double p_detection = 
      exp( -powf(nearest_strip - x, 2) / 
            (2 * (sigma2 + fiducialSigma2))) /
      (sqrtf(2 * 3.1415 * (sigma2 + fiducialSigma2)));

    if (stopped_ || config_.fiducial_FP_rate > p_detection){
      // no change, ignore this
      printf("ignoring strip, estimated pos %f + %f, %f\n", nearest_strip - projected_travel, projected_travel, p_detection);
    } else {
      printf("updating to nearest strip %f + %f, p %f\n", nearest_strip - projected_travel, projected_travel, p_detection);
      // update to the product distribution
      state_[0].mu[0] = (fiducialSigma2 * x + sigma2 * nearest_strip) / (fiducialSigma2 + sigma2);
      state_[0].Sigma[0][0] = sqrtf( sigma2 * fiducialSigma2 / (sigma2 + fiducialSigma2) );
      // velocity too if reasonable
      if (estimated_speed > 0.0 and fabs(estimated_speed - state_[0].mu[1]) < config_.fiducial_max_vel_difference){
        state_[0].mu[1] = (fiducialSigmaVel2 * xd + sigmaVel2 * estimated_speed) / (fiducialSigmaVel2 + sigmaVel2);
        state_[0].Sigma[1][1] = sqrtf( sigmaVel2 * fiducialSigmaVel2 / (sigmaVel2 + fiducialSigmaVel2) );
        printf("Updating estimated speed to %f\n", estimated_speed);
      }
    }
  }
  do_fiducial_update_ = false;
  return true;
}

void StateEstimator::publish(double t) {
  mithl_state_estimator_particle_set msg;
  msg.utime = t*1E6;
  msg.n_particles = 0;
  // make state workspace a continuous array of n_particles particle structures
  for (int i=0; i<config_.max_particles; i++){
    if (state_[i].id >= 0){
      state_workspace_[msg.n_particles] = state_[i];
      msg.n_particles += 1;
    }
  }
  // use that to construct set message
  msg.particles = &(state_workspace_[0]);
  mithl_state_estimator_particle_set_publish(zcm_, "_FC_SE", &msg);
}

void StateEstimator::publishDetails(double t) {
  mithl_state_estimator_state_t msg;
  msg.utime = t*1E6;
  msg.front_nav_imu_good = front_nav_imu_good_;
  msg.rear_nav_imu_good = rear_nav_imu_good_;
  msg.stopped = stopped_;
  msg.stopping_detector_mean_est = last_stopping_detector_mean_est_;
  msg.stopping_detector_var_est = last_stopping_detector_max_var_est_;
  msg.rear_imu_x_bias = rear_imu_x_bias_;
  msg.front_imu_x_bias = front_imu_x_bias_;
  mithl_state_estimator_state_t_publish(zcm_, "_FC_SE_STATE", &msg);
}

void StateEstimator::fiducial_t_handler(const mithl_fiducial_t * msg){
  // sanity check ranges first
  if (msg->time_since_last >= 0 && msg->average_time_strip >= 0 && msg->average_time_strip <= 1.0){
    last_fiducial_time_ = get_current_time(); 
    if (last_time_since_last_strip_ > 0 && msg->time_since_last < last_time_since_last_strip_){
      do_fiducial_update_ = true;
      last_average_time_strip_ = msg->average_time_strip;
      printf("got fiducial time %f vs %f\n", msg->time_since_last, last_time_since_last_strip_);
    }
    last_time_since_last_strip_ = msg->time_since_last;
  }
}


void StateEstimator::state_estimator_config_t_handler(const mithl_state_estimator_config_t * msg){
  memcpy(&config_, msg, sizeof(mithl_state_estimator_config_t));
}
