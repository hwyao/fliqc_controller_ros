#include "fliqc_controller_ros/fliqc_state_source_bridge.hpp"

namespace fliqc_controller_ros
{
    FrankaModelInterfaceBridge::FrankaModelInterfaceBridge(franka_hw::FrankaModelInterface* model_interface, const std::string& arm_id) {
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        } catch (const hardware_interface::HardwareInterfaceException& ex) { // Ensure exception type is correct
            throw std::runtime_error("FrankaModelInterfaceBridge: Exception getting model handle from interface: " + std::string(ex.what()));
        }
    }

    void FrankaModelInterfaceBridge::getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        std::array<double, 49> mass = model_handle_->getMass();
        result = Eigen::Matrix<double, 7, 7>(mass.data());
    }
} // namespace fliqc_controller_ros