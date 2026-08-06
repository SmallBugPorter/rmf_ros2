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
        "compliant" : "greedy";

      const bool compliant_success = result.compliant_job
        && result.compliant_job->progress().success();
      const auto selected_type = compliant_success ?
        "compliant" : (result.greedy_job ? "greedy" : "none");

      std::ostringstream status;
      status << std::boolalpha
             << "[FindPath] participant=" << participant_id
             << ", Result::type=" << result_type
             << ", selected=" << selected_type
             << ", compliant={available="
             << static_cast<bool>(result.compliant_job);

      if (result.compliant_job)
      {
        const auto& progress = result.compliant_job->progress();
        status << ", success=" << progress.success()
               << ", saturated=" << progress.saturated()
               << ", interrupted=" << progress.interrupted()
               << ", cost_estimate=";

        if (const auto cost = progress.cost_estimate())
          status << *cost;
        else
          status << "null";

        status << ", maximum_cost_estimate=";
        if (const auto maximum =
          progress.options().maximum_cost_estimate())
        {
          status << *maximum;
        }
        else
        {
          status << "null";
        }
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
