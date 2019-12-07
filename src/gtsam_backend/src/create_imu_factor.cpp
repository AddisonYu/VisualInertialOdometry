#include "GraphSolver.h"
gtsam::CombinedImuFactor create_imu_factor(double updatetime, gtsam::Values& values_initial){
	gtsam::CombinedImuFactor imu_factor(gtsam::Values& values_initial.at<Pose3>(X(ct_state-1)),
					result.at<Vector3>(V(ct_state-1)),
				       	gtsam::Values& values_initial.at<Pose3>(X(ct_state)),
					result.at<Vector3>(V(ct_state)),
					result.at<imuBias::ConstantBias>(B(ct_state-1)),
					result.at<imuBias::ConstantBias>(B(ct_state)),
                                   *preint_imu_combined);
	graph->add(imu_factor);
	return CombinedImuFactor;
}







