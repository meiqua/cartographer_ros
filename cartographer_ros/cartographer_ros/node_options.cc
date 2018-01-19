/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

namespace cartographer_ros {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename,
    const std::string& init_pose_filename) {

    auto file_resolver = cartographer::common::make_unique<
    cartographer::common::ConfigurationFileResolver>(
    std::vector<std::string>{configuration_directory});
    const std::string code =
    file_resolver->GetFileContentOrDie(configuration_basename);

  std::string initial_pose =
    file_resolver->GetFileContentOrDie(init_pose_filename);
    
    auto lua_parameter_dictionary =
    cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          code, std::move(file_resolver));

    auto initial_trajectory_pose_file_resolver =
        cartographer::common::make_unique<
            cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    auto initial_trajectory_pose =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(
            "return " + initial_pose,
            std::move(initial_trajectory_pose_file_resolver));

  return std::make_tuple(CreateNodeOptions(lua_parameter_dictionary.get()),
                              CreateTrajectoryOptions(lua_parameter_dictionary.get(),
                                   initial_trajectory_pose.get()));
}

}  // namespace cartographer_ros
