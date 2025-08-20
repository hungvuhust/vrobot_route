#pragma once

#include <drogon/drogon.h>
#include <drogon/orm/DbClient.h>
#include <drogon/orm/Mapper.h>

#include "models/Curvelink.h"
#include "models/Map.h"
#include "models/Node.h"
#include "models/Straightlink.h"

using namespace drogon::orm;
using namespace drogon_model::amr_01::amr_ros2;

namespace vrobot_route {

class DbManager {
public:
  explicit DbManager(const std::string &connection_info);
  ~DbManager();

  Map                       getMap(const std::string &map_name);
  std::vector<Node>         getNodes(const std::string &map_name);
  std::vector<Straightlink> getStraightlinks(const std::string &map_name);
  std::vector<Curvelink>    getCurvelinks(const std::string &map_name);

  using Ptr = std::shared_ptr<DbManager>;

private:
  std::shared_ptr<drogon::orm::DbClient> db_client_;
};

}  // namespace vrobot_route