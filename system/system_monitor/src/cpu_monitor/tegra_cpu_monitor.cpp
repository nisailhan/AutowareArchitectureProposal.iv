// Copyright 2020 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file tegra_cpu_monitor.cpp
 * @brief TEGRA PU monitor class
 */

#include <string>
#include <vector>

#include "system_monitor/cpu_monitor/tegra_cpu_monitor.hpp"
#include "system_monitor/system_monitor_utility.hpp"

CPUMonitor::CPUMonitor(const rclcpp::NodeOptions & options)
: CPUMonitorBase("cpu_monitor", options)
{
  // There is no event record for thermal throttling.
  // Need to manually monitor temperature to figure out if thermal limits crossed or not.
  updater_.removeByName("CPU Thermal Throttling");
}

void CPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat) {}

void CPUMonitor::getTempNames()
{
  // Jetson TX1 TX2 Nano: thermal_zone1, Xavier: thermal_zone0
  std::vector<thermal_zone> therms;
  SystemMonitorUtility::getThermalZone("CPU-therm", &therms);

  for (auto itr = therms.begin(); itr != therms.end(); ++itr) {
    temps_.emplace_back(itr->label_, itr->path_);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CPUMonitor)
