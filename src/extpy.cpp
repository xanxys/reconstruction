#include "extpy.h"

#include <jsoncpp/json/json.h>

#include <program_proxy.h>

namespace recon {

std::vector<Eigen::Vector2f> extFixContour(
		const std::vector<Eigen::Vector2f>& poly,
		const boost::optional<std::string>& vis_path) {
	Json::Value fc_arg;
	Json::Value poly_json;
	for(const auto& pt : poly) {
		Json::Value pt_json;
		pt_json.append(pt(0));
		pt_json.append(pt(1));
		poly_json.append(pt_json);
	}
	fc_arg["points"] = poly_json;
	if(vis_path) {
		fc_arg["vis_path"] = *vis_path;
	}
	const Json::Value contour_result = call_external("extpy/fix_contour.py", fc_arg);
	std::vector<Eigen::Vector2f> contour_points;
	for(const auto& pt_json : contour_result["points"]) {
		contour_points.emplace_back(
			pt_json[0].asDouble(), pt_json[1].asDouble());
	}
	return contour_points;
}

}  // namespace
