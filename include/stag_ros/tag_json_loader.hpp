#pragma once
#include <fstream>
#include <jsoncpp/json/json.h>
#include <opencv2/core/types.hpp>
#include "structures.hpp"
namespace stag_ros {

namespace tag_json_loader {

inline bool checkIdAndFrameValid(const std::vector<Tag> &tags,
                                 const std::vector<Bundle> &bundles,
                                 const std::string frame, const int id) {
  for (auto tag : tags) {
    if (tag.id == id or tag.frame_id.compare(frame) == 0) return false;
  }
  for (auto bundle : bundles) {
    if (bundle.frame_id.compare(frame) == 0) return false;
    for (auto tag : bundle.tags) {
      if (tag.id == id) return false;
    }
  }
  return true;
}

inline void load(const std::string &filename, std::vector<Tag> &tags,
                 std::vector<Bundle> &bundles) {
  tags.clear();
  bundles.clear();

  Json::Value cfg_root;
  std::ifstream cfgfile(filename);
  cfgfile >> cfg_root;

  if (cfg_root.isMember("tags")) {
    std::cout << "Loading individual tags" << std::endl;
    for (auto frame_id : cfg_root["tags"].getMemberNames()) {
      Tag t;
      t.id = cfg_root["tags"][frame_id]["id"].asInt();
      t.frame_id = frame_id;
      if (!checkIdAndFrameValid(tags, bundles, frame_id, t.id)) {
        std::cerr << "Tag frame_id or id duplicated at tag: " << frame_id
                  << " with id: " << t.id << std::endl;
        exit(EXIT_FAILURE);
      }
      std::cout << "loading " << t.frame_id<<std::endl;
      for (int i=0; i < 3; ++i) {
        t.corners[i] =
            cv::Point3d(cfg_root["tags"][frame_id]["corners"][i][0].asDouble(),
                        cfg_root["tags"][frame_id]["corners"][i][1].asDouble(),
                        cfg_root["tags"][frame_id]["corners"][i][2].asDouble());
        std::cout << t.corners[i] << std::endl;
      }
      t.center = (t.corners[2]+t.corners[0])/2;
      // project the vector C1->C2 from C0 to get the fourth corner
      t.corners[3] = t.corners[0] + (t.corners[2] - t.corners[1]);
      tags.emplace_back(t);
    }
  }
  if (cfg_root.isMember("bundles")) {
    for (auto frame_id : cfg_root["bundles"].getMemberNames()) {
      Bundle b;
      b.frame_id = frame_id;
      for (auto tit : cfg_root["bundles"][frame_id]) {
        Tag t;
        t.id = tit["id"].asInt();

        if (!checkIdAndFrameValid(tags, bundles, frame_id, t.id)) {
          std::cerr << "Tag frame_id or id duplicated at tag: " << frame_id
                    << " with id: " << t.id << std::endl;

          exit(EXIT_FAILURE);
        }
        for (size_t i=1; i < 4; ++i)
          t.corners[i] = cv::Point3d(
              cfg_root["tags"][frame_id]["corners"][0][0].asDouble(),
              cfg_root["tags"][frame_id]["corners"][0][1].asDouble(),
              cfg_root["tags"][frame_id]["corners"][0][2].asDouble());
        // project the vector C2->C3 from C1 to get C4
        t.corners[0]=(t.corners[3] + t.corners[1])/2;
        t.corners[4] = t.corners[1] + (t.corners[3] - t.corners[2]);
        b.tags.emplace_back(t);
      }
      bundles.emplace_back(b);
    }
  }
}

} // namespace tagJsonLoader
}  // namespace stag_ros