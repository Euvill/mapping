#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_ODO_PRE_INTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_ODO_PRE_INTEGRATION_HPP_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp"

#include <g2o/core/base_binary_edge.h>

#include "glog/logging.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace g2o {

class EdgePRVAGodoPreIntegration : public g2o::BaseBinaryEdge<6, Vector6d, g2o::VertexPRVAG, g2o::VertexPRVAG> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgePRVAGodoPreIntegration()
	 : g2o::BaseBinaryEdge<6, Vector6d, g2o::VertexPRVAG, g2o::VertexPRVAG>() {
	 }

	virtual void computeError() override {
        g2o::VertexPRVAG* v0 = dynamic_cast<g2o::VertexPRVAG*>(_vertices[0]);
        g2o::VertexPRVAG* v1 = dynamic_cast<g2o::VertexPRVAG*>(_vertices[1]);

		const Eigen::Vector3d &pos_i = v0->estimate().pos;
		const Sophus::SO3d    &ori_i = v0->estimate().ori;

		const Eigen::Vector3d &pos_j = v1->estimate().pos;
		const Sophus::SO3d    &ori_j = v1->estimate().ori;

		const Eigen::Vector3d &pos_ij   = _measurement.block<3, 1>(0, 0);
		const Eigen::Vector3d &theta_ij = _measurement.block<3, 1>(3, 0);

		_error.block<3, 1>(0, 0) = ori_i.matrix().transpose() * (pos_j - pos_i) - pos_ij;
		_error.block<3, 1>(3, 0) = (Sophus::SO3d::exp(theta_ij).inverse() * ori_i.inverse() * ori_j).log();
    }

	void setT(const double &T) {
		T_ = T;
	}

    virtual void setMeasurement(const Vector6d& m) override {
		_measurement = m;
	}

	virtual bool read(std::istream& is) override {
		double T;
		is >> T;
		
		Vector6d v;
		is >> v(0 + 0) >> v(0 + 1) >> v(0 + 2)
		   >> v(3 + 0) >> v(3 + 1) >> v(3 + 2);

		setT(T);

		for (int i = 0; i < information().rows(); ++i) {
			for (int j = i; j < information().cols(); ++j) {
				is >> information()(i, j);
				// update cross-diagonal element:
				if (i != j) {
					information()(j, i) = information()(i, j);
				}
			}
		}

		return true;
	}

	virtual bool write(std::ostream& os) const override {
    	Vector6d v = _measurement;
		
		os << T_ << " ";

		os << v(0 + 0) << " " << v(0 + 1) << " " << v(0 + 2) << " "
		   << v(3 + 0) << " " << v(3 + 1) << " " << v(3 + 2) << " ";

		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);

		return os.good();
	}

private:
	double T_ = 0.0;
};

} // namespace g2o

#endif // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_IMU_PRE_INTEGRATION_HPP_
