/*
 *  Input.cpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 * 
 * Modified on: Dec 11. 2025
 *      Author: Hao Hung
 *   Institute: DRIC
 */

#include <memory>

#include "elevation_mapping/input_sources/Input.hpp"

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

Input::Input(rclcpp::Node::SharedPtr nh) : nodeHandle_(nh) {}

bool Input::configure(std::string name, const std::string& SourceConfigurationName,
                      const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  // Configuration Guards.

  Parameters parameters;

  // Check Optional enabled parameter.
  rclcpp::Parameter param;
  if (nodeHandle_->has_parameter(SourceConfigurationName + ".enabled")) 
  {
    nodeHandle_->get_parameter(SourceConfigurationName + ".enabled", param);
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      RCLCPP_ERROR(nodeHandle_->get_logger(),
                   "Could not configure input source %s because parameter 'enabled' has the wrong type.",
                   name.c_str());
      return false;
    }
    parameters.isEnabled_ = param.as_bool();
  }

  // Required parameters: type, topic, queue_size, publish_on_update, sensor_processor.type
  std::vector<std::pair<std::string, rclcpp::ParameterType>> requiredParameters = {
      {".type", rclcpp::ParameterType::PARAMETER_STRING},
      {".topic", rclcpp::ParameterType::PARAMETER_STRING},
      {".queue_size", rclcpp::ParameterType::PARAMETER_INTEGER},
      {".publish_on_update", rclcpp::ParameterType::PARAMETER_BOOL},
      {".sensor_processor.type", rclcpp::ParameterType::PARAMETER_STRING}};

  for (const auto& [paramName, paramType] : requiredParameters) {
    if (!nodeHandle_->has_parameter(SourceConfigurationName + paramName)) 
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(),
                   "Could not configure input source %s because parameter '%s' was not given.",
                   name.c_str(), paramName.c_str());
      return false;
    }
    nodeHandle_->get_parameter(SourceConfigurationName + paramName, param);
    if (param.get_type() != paramType) 
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(),
                   "Could not configure input source %s because parameter '%s' has the wrong type.",
                   name.c_str(), paramName.c_str());
      return false;
    }
  }

  parameters.name_ = name;
  nodeHandle_->get_parameter(SourceConfigurationName + ".type", param);
  parameters.type_ = param.as_string();

  nodeHandle_->get_parameter(SourceConfigurationName + ".topic", param);
  parameters.topic_ = param.as_string();

  nodeHandle_->get_parameter(SourceConfigurationName + ".queue_size", param);
  const int queueSize = param.as_int();
  if (queueSize < 0) 
  {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "The specified queue_size is negative.");
    return false;
  }
  parameters.queueSize_ = static_cast<uint32_t>(queueSize);


  nodeHandle_->get_parameter(SourceConfigurationName + ".publish_on_update", param);
  parameters.publishOnUpdate_ = param.as_bool();

  parameters_.setData(parameters);

  // SensorProcessor
  if (!configureSensorProcessor(name, SourceConfigurationName, generalSensorProcessorParameters)) 
  {
    return false;
  }
  RCLCPP_INFO(nodeHandle_->get_logger(),
              "Configured %s:%s @ %s (publishing_on_update: %s), using %s to process data.",
              parameters.type_.c_str(),
              parameters.name_.c_str(),
              parameters.topic_.c_str(),
              parameters.publishOnUpdate_ ? "true" : "false",
              sensorProcessor_->getType().c_str());

  return true;
}

std::string Input::getSubscribedTopic() const {
  const Parameters parameters{parameters_.getData()};
  return parameters.topic_;
}

bool Input::configureSensorProcessor(std::string name, const std::string& parameter,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters)  
{
  rclcpp::Parameter param;
  nodeHandle_->get_parameter(parameter + ".sensor_processor.type", param);
  std::string sensorType = param.as_string();

  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_shared<StructuredLightSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_shared<StereoSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_shared<LaserSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_shared<PerfectSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(),
                 "The sensor type %s is not available for input source %s.",
                 sensorType.c_str(), name.c_str());
    return false;
  }

  return sensorProcessor_->readParameters();
}

}  // namespace elevation_mapping
