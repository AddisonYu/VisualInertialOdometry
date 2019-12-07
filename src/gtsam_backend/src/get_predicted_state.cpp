
#include "GraphSolver.h"
gtsam::State get_predicted_state(gtsam::Values& values_initial){
      // Overwrite the beginning of the preintegration for the next step.
	prev_state =  gtsam::State(gtsam::Values& values_initial.at<Pose3>(X(ct_state)),
                            result.at<Vector3>(V(ct_state)));
	prev_bias = result.at<imuBias::ConstantBias>(B(ct_state));



    // Now optimize and compare results.
	 gtsam::State prop_state = imu_preintegrated_->predict(prev_state, prev_bias);


	return prop_state;
}




