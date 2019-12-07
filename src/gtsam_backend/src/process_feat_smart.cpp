
#include "GraphSolver.h"

//using namespace gtsam;


void process_feat_smart(double timestamp, std::vector<unit> leftids, std::vector<Eigen::Vecttor 2d> leftuv {
	// Loop through left features
	for (int i=0; i< leftids.size(); ++i)
		// check if the id is already in the graph
		if (measurement_smart_lookup_left.find(leftids.at(i)) !=measurement_smart_lookup_left.end()){
			measurement_smart_lookup_left[leftids.at(i)]->add(gtsam::Poin2(leftuv.at(i)),X(ct_state));
			continue;
		}
		// Set up noise and camera calibration
		auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2,config->sigma_camera);
		gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());

		//transform from camera frame to Imu frame(sensor:camera, body:imu)
		gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI),gtsam::point3(config->p_IinC0));

		//Create smart factor with measurement from first pose only
		SmartFactor::shared_ptr smartFactor(new SmartFactor(measurementNoise, K, sensor_P_body.inverse()));
		smartFactor->add(gtsam::Point2(leftuv.at(i)),X(ct_state));

		//create a hashtable for smart factor
		measurement_smart_lookup_left[leftids.at(i)]=smartFactor;

		//Add smart factor to graph
		graph_new-> push_back(smartFactor);
		graph->push_back(smartFactor);
	}
}


		















