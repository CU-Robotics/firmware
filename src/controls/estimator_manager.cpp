#include "estimator_manager.hpp"
#include "estimator.hpp"
#include "robot_state_map.hpp"


EstimatorManager::EstimatorManager() { }

EstimatorManager::~EstimatorManager() {
    estimators.clear();
}

void EstimatorManager::init(
        const std::vector<Cfg::Estimator>& estimator_configurations, 
        SensorManager& sensor_manager, CANManager& can) 
{
    available_states.clear();
    for (uint32_t i = 0; i < (uint32_t)(Cfg::StateName::StateNameCount); i++) {
        available_states.push_back(static_cast<Cfg::StateName>(i));
    }
    for (const Cfg::Estimator& estimator_config : estimator_configurations) {
        init_estimator(estimator_config, sensor_manager, can);
    }
}

void EstimatorManager::init_estimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can) {
    switch (estimator_config.estimator_type) {
    case Cfg::EstimatorType::GimbalAndChassis:
        estimators.push_back(std::make_unique<GimbalAndChassisEstimator>(estimator_config, sensor_manager, can, available_states));
        break;
    case Cfg::EstimatorType::FlywheelVelocity:
        estimators.push_back(std::make_unique<FlywheelEstimator>(estimator_config, sensor_manager, can, available_states));
        break;
    case Cfg::EstimatorType::FeederPosition:
        estimators.push_back(std::make_unique<NewFeederEstimator>(estimator_config, sensor_manager, can, available_states));
        break;
    default:
        break;
    }
}

void EstimatorManager::step(RobotStateMap& current_state_map, int override) {
    RobotStateMap previous_state_map = current_state_map;

    for (auto& estimator : estimators) {
        estimator->step_states(current_state_map, previous_state_map, override);
    }
}
