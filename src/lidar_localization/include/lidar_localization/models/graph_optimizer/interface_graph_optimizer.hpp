/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:35:19
 */

#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_

#include <string>
#include <deque>

#include <Eigen/Dense>

// data input:
#include "lidar_localization/sensor_data/key_frame.hpp"

#include "lidar_localization/models/pre_integrator/imu_pre_integrator.hpp"
#include "lidar_localization/models/pre_integrator/odo_pre_integrator.hpp"


namespace lidar_localization {

class InterfaceGraphOptimizer {
public:
    virtual ~InterfaceGraphOptimizer() {}

    // 优化
    virtual bool Optimize() = 0;
    
    // 输入、输出数据
    virtual int GetNodeNum() = 0;
    // 添加节点、边、鲁棒核
    virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;
    
    // LIO state:
    virtual bool GetOptimizedKeyFrame(std::deque<KeyFrame> &optimized_key_frames) = 0;
    /**
     * @brief  add vertex for LIO key frame
     * @param  lio_key_frame, LIO key frame with (pos, ori, vel, b_a and b_g)
     * @param  need_fix, shall the vertex be fixed to eliminate trajectory estimation ambiguity
     * @return true if success false otherwise
     */
    virtual void AddPRVAGNode(
      const KeyFrame &lio_key_frame, const bool need_fix
    ) = 0;
    virtual void AddPRVAGRelativePoseEdge(
      const int vertex_index_i, const int vertex_index_j,
      const Eigen::Matrix4d &relative_pose, const Eigen::VectorXd &noise
    ) = 0;
    virtual void AddPRVAGPriorPoseEdge(
      const int vertex_index,
      const Eigen::Vector3d &pos, const Eigen::Vector3d &noise
    ) = 0;
    virtual void AddPRVAGIMUPreIntegrationEdge(
      const int vertex_index_i, const int vertex_index_j,
      const IMUPreIntegrator::IMUPreIntegration &imu_pre_integration
    ) = 0;
    virtual void AddPRVAGODOPreIntegrationEdge(
      const int vertex_index_i, const int vertex_index_j,
      const ODOPreIntegrator::ODOPreIntegration &odo_pre_integration
    ) = 0;
    void SetMaxIterationsNum(int max_iterations_num);
  
protected:
    int max_iterations_num_ = 512;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_