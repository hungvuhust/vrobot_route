#include "vrobot_route/db_client.hpp"
#include "Node.h"

namespace vrobot_route {

DbManager::DbManager(const std::string &connection_info) {
  db_client_ = drogon::orm::DbClient::newPgClient(connection_info, 1);
}

DbManager::~DbManager() {}

Map DbManager::getMap(const std::string &map_name) {
  try {
    drogon::orm::Mapper<Map> map_mapper(db_client_);
    auto                     criteria = drogon::orm::Criteria(
                            drogon_model::amr_01::amr_ros2::Map::Cols::_map_name,
                            drogon::orm::CompareOperator::EQ, map_name);
    auto map = map_mapper.findBy(criteria);

    if (map.empty()) {
      throw std::runtime_error("Map not found");
    }

    return map.front();
  } catch (const drogon::orm::DrogonDbException &e) {
    throw std::runtime_error("Failed to get map from database: " +
                             std::string(e.base().what()));
  }
}

std::vector<Node> DbManager::getNodes(const std::string &map_name) {
  try {
    auto map = getMap(map_name);

    drogon::orm::Mapper<Node> node_mapper(db_client_);
    auto                      criteria = drogon::orm::Criteria(
                             drogon_model::amr_01::amr_ros2::Node::Cols::_map_id,
                             drogon::orm::CompareOperator::EQ, *map.getIdMap());
    auto nodes = node_mapper.findBy(criteria);
    return nodes;
  } catch (const drogon::orm::DrogonDbException &e) {
    throw std::runtime_error("Failed to get nodes from database: " +
                             std::string(e.base().what()));
  }
}

std::vector<Straightlink>
DbManager::getStraightlinks(const std::string &map_name) {
  try {
    auto map = getMap(map_name);

    drogon::orm::Mapper<Straightlink> straightlink_mapper(db_client_);
    auto                              criteria = drogon::orm::Criteria(
                                     drogon_model::amr_01::amr_ros2::Straightlink::Cols::_map_id,
                                     drogon::orm::CompareOperator::EQ, *map.getIdMap());
    auto straightlinks = straightlink_mapper.findBy(criteria);
    return straightlinks;
  } catch (const drogon::orm::DrogonDbException &e) {
    throw std::runtime_error("Failed to get straightlinks from database: " +
                             std::string(e.base().what()));
  }
}

std::vector<Curvelink> DbManager::getCurvelinks(const std::string &map_name) {
  try {
    auto map = getMap(map_name);

    drogon::orm::Mapper<Curvelink> curvelink_mapper(db_client_);
    auto                           criteria = drogon::orm::Criteria(
                                  drogon_model::amr_01::amr_ros2::Curvelink::Cols::_map_id,
                                  drogon::orm::CompareOperator::EQ, *map.getIdMap());
    auto curvelinks = curvelink_mapper.findBy(criteria);
    return curvelinks;
  } catch (const drogon::orm::DrogonDbException &e) {
    throw std::runtime_error("Failed to get curvelinks from database: " +
                             std::string(e.base().what()));
  }
}

} // namespace vrobot_route