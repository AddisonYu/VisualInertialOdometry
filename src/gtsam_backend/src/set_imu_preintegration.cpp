

#include "GraphSolver.h"
bool set_imu_preintegration(const gtsam::State& prior_state){
	Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * config->sigma_a_sq;
	Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * config->sigma_g_sq;
	Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
	Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * config->sigma_wa_sq;
	Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * config->sigma_wg_sq;
	Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedU(congif->gravity(2));
  // PreintegrationBase params:
	p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
	p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
	p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
	p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
	p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
	p->biasAccOmegaInt = bias_acc_omega_int;		
	preint_gtsam = new PreintegratedCombinedMeasurements(p,prior_state.b());
}

gtsam::State get_predicted_state(gtsam::Values& values_initial){
	//get the current state (K)
	gtsam::State stateK = gtsam::State(values_initial.at<gtsam::Pose3>(X(ct_state)),
					  (values_initial.at<gtsam::Vector3>(V(ct_state)),
					  (values__initial.at<gtsam::bias>(B(ct_state)));
	gtsam::State stateK1 = preint_gtsam->predict(gtsam::NavState(stateK.pose(), stateK.v()),stateK.b());
	return stateK1;
}


void reset_imu_integration(){
	preint_gtsam->resetIntegrationAndSetBias(value_initial.at<gtsam::Bias>(B(ct_state)));
}




