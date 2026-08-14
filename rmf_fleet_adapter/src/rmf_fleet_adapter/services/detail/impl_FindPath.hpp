/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__PLANNING_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__PLANNING_HPP

#include "../FindPath.hpp"

#include <iostream>
#include <sstream>

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
template<typename Subscriber>
void FindPath::operator()(const Subscriber& s)
{
  _search_sub = rmf_rxcpp::make_job<jobs::SearchForPath::Result>(_search_job)
    .observe_on(rxcpp::observe_on_event_loop())
    .subscribe(
    [s, participant_id = _participant_id](
      const jobs::SearchForPath::Result& result)
    {
      const auto result_type =
        result.type == jobs::SearchForPath::Type::compliant ?
        "合规规划" : "贪心规划";

      const bool compliant_success = result.compliant_job
        && result.compliant_job->progress().success();
      const auto selected_type = compliant_success ?
        "合规规划" : (result.greedy_job ? "贪心规划" : "无");

      const auto bool_to_zh = [](const bool value)
        {
          return value ? "是" : "否";
        };

      std::ostringstream status;
      status << "[路径规划] 参与者=" << participant_id
             << ", 返回类型=" << result_type
             << ", 已选择=" << selected_type
             << ", 合规规划={可用="
             << bool_to_zh(static_cast<bool>(result.compliant_job));

      if (result.compliant_job)
      {
        const auto& progress = result.compliant_job->progress();
        status << ", 成功=" << bool_to_zh(progress.success())
               << ", 搜索饱和=" << bool_to_zh(progress.saturated())
               << ", 已中断=" << bool_to_zh(progress.interrupted())
               << ", 成本估算=";

        if (const auto cost = progress.cost_estimate())
          status << *cost;
        else
          status << "无";

        status << ", 最大成本估算=";
        if (const auto maximum =
          progress.options().maximum_cost_estimate())
        {
          status << *maximum;
        }
        else
        {
          status << "无";
        }

        status << ", 阻塞参与者=[";
        const auto blockers = progress.blockers();
        const auto& validator = progress.options().validator();
        const auto* schedule_validator =
          dynamic_cast<const rmf_traffic::agv::ScheduleRouteValidator*>(
          validator.get());

        for (std::size_t i = 0; i < blockers.size(); ++i)
        {
          if (i > 0)
            status << ", ";

          const auto blocker_id = blockers[i];
          status << "{id=" << blocker_id;

          if (schedule_validator)
          {
            const auto description = schedule_validator->schedule_viewer()
              .get_participant(blocker_id);
            if (description)
            {
              status << ", owner=" << description->owner()
                     << ", name=" << description->name();
            }
          }

          status << "}";
        }

        status << "]";
      }

      status << "}";
      std::cout << status.str() << std::endl;

      // The first time we get a result back, it will be when the jobs are
      // completed.
      if (compliant_success)
      {
        s.on_next(result.compliant_job->progress());
        s.on_completed();
      }
      else if (result.greedy_job)
      {
        // We will send back the result of the greedy job, whether or not it
        // successfully found a plan. It is up to the subscriber to check
        // whether it was successful and decide what to do about failures.
        s.on_next(result.greedy_job->progress());
        s.on_completed();
      }
      else
      {
        s.on_error(std::make_exception_ptr(
          std::runtime_error(
            "[FindPath] Unexpected result from SearchForPath")));
      }
    },
    [s](std::exception_ptr e)
    {
      s.on_error(e);
    },
    [s]()
    {
      // If this is triggered without a result coming in, that implies that the
      // job was impossible.
      s.on_completed();
    });
}

}
}

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__PLANNING_HPP
