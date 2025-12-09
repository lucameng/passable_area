#ifndef TRAVERSAL_COST_HPP
#define TRAVERSAL_COST_HPP

#include "common_types.hpp"

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

class ElevationMap;

class TraversalCost {
public:
  TraversalCost(ElevationMap &map, const TraversalCostParams &params);

  const TraversalCostParams &getParams() const noexcept { return params_; }

  bool updateCostLayer();
  bool publish(const ros::Publisher &publisher, const std::string &frame_id,
               const ros::Time &stamp) const;

private:
  bool fillMessage(nav_msgs::OccupancyGrid &msg, const std::string &frame_id,
                   const ros::Time &stamp) const;

  ElevationMap &map_;
  TraversalCostParams params_;
};

#endif // TRAVERSAL_COST_HPP
