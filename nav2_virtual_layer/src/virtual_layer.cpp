/*********************************************************************
 * Copyright (C) Sherif Fathey - All Rights Reserved
 *
 * This file is part of the nav2_virtual_layer package.
 *
 * @file virtual_layer.cpp
 * @brief Implementation of the Virtual Layer plugin for Nav2
 * @author Sherif Fathey
 *********************************************************************/

#include "nav2_virtual_layer/virtual_layer.hpp"

namespace nav2_virtual_layer {

// =======================
// Constructor
// =======================
VirtualLayer::VirtualLayer()
    : enabled_(true), map_frame_("map"), tf_buffer_(nullptr) {}

// =======================
// UUID Generator
// =======================
std::string VirtualLayer::generateUUID() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<> dis(0, 15);

  const char *hex_chars = "0123456789abcdef";
  std::string uuid = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx";

  for (auto &c : uuid) {
    if (c == 'x') {
      c = hex_chars[dis(gen)];
    } else if (c == 'y') {
      c = hex_chars[(dis(gen) & 0x3) | 0x8];
    }
  }
  return uuid;
}

// =======================
// String Trimming Helper
// =======================
std::string trim(const std::string &str) {
  size_t first = str.find_first_not_of(" \t\n\r");
  if (first == std::string::npos)
    return "";
  size_t last = str.find_last_not_of(" \t\n\r");
  return str.substr(first, last - first + 1);
}

// =======================
// Check and Remove Expired Shapes
// =======================
void VirtualLayer::checkExpiredShapes() {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  rclcpp::Time current_time = node->now();

  // {uuid, type}
  std::vector<std::pair<std::string, std::string>> expired_shapes;

  // =====================
  // Check expired circles
  // =====================
  for (const auto &[uuid, circle] : circles_) {
    if (circle.isExpired(current_time)) {
      expired_shapes.emplace_back(uuid, "CIRCLE");
      RCLCPP_DEBUG(logger_, "Circle [%s] expired after %.1f seconds",
                   uuid.c_str(), circle.duration_seconds);
    }
  }

  // =====================
  // Check expired lines
  // =====================
  for (const auto &[uuid, line] : lines_) {
    if (line.isExpired(current_time)) {
      expired_shapes.emplace_back(uuid, "LINE");
      RCLCPP_DEBUG(logger_, "Line [%s] expired after %.1f seconds",
                   uuid.c_str(), line.duration_seconds);
    }
  }

  // =====================
  // Check expired polygons
  // =====================
  for (const auto &[uuid, polygon] : polygons_) {
    if (polygon.isExpired(current_time)) {
      expired_shapes.emplace_back(uuid, "POLYGON");
      RCLCPP_DEBUG(logger_, "Polygon [%s] expired after %.1f seconds",
                   uuid.c_str(), polygon.duration_seconds);
    }
  }

  // =====================
  // Remove expired shapes
  // =====================
  for (const auto &[uuid, type] : expired_shapes) {
    circles_.erase(uuid);
    lines_.erase(uuid);
    polygons_.erase(uuid);

    RCLCPP_INFO(logger_, "Removed expired %s [%s]", type.c_str(), uuid.c_str());
  }

  // =====================
  // Summary
  // =====================
  if (!expired_shapes.empty()) {
    RCLCPP_INFO(logger_, "Removed %zu expired shape(s)", expired_shapes.size());
  }
}

// =======================
// Resolve Forms File Path
// =======================
std::string VirtualLayer::resolveFormsFilePath(const std::string &forms_file) {
  auto node = node_.lock();

  // 1: Absolute Path
  if (forms_file[0] == '/') {
    RCLCPP_DEBUG(logger_, "Using absolute path: %s", forms_file.c_str());
    return forms_file;
  }

  // 2: package:// URI
  if (forms_file.find("package://") == 0) {
    size_t pkg_start = 10;
    size_t slash_pos = forms_file.find('/', pkg_start);

    if (slash_pos != std::string::npos) {
      std::string package_name =
          forms_file.substr(pkg_start, slash_pos - pkg_start);
      std::string relative_path = forms_file.substr(slash_pos + 1);

      try {
        std::string package_share =
            ament_index_cpp::get_package_share_directory(package_name);
        std::string full_path = package_share + "/" + relative_path;

        RCLCPP_DEBUG(logger_, "Resolved package:// URI '%s' to: %s",
                     forms_file.c_str(), full_path.c_str());
        return full_path;

      } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Failed to find package '%s': %s",
                     package_name.c_str(), e.what());
        return "";
      }
    }
  }

  // 3: Relative Path within Package
  try {
    std::string package_share =
        ament_index_cpp::get_package_share_directory("nav2_virtual_layer");

    std::string full_path;

    if (forms_file.find('/') != std::string::npos) {
      full_path = package_share + "/" + forms_file;
    } else {
      full_path = package_share + "/config/" + forms_file;
    }

    RCLCPP_DEBUG(logger_, "Resolved relative path '%s' to: %s",
                 forms_file.c_str(), full_path.c_str());
    return full_path;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Could not find package share directory: %s",
                 e.what());
    return "";
  }
}

// =======================
// Load Forms from YAML
// =======================
void VirtualLayer::loadFormsFromYAML(const std::string &yaml_path) {
  auto node = node_.lock();

  std::ifstream file(yaml_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open file: %s", yaml_path.c_str());
    return;
  }

  int default_cost =
      node->get_parameter(name_ + ".default_cost_level").as_int();
  std::vector<std::string> forms;
  std::string line;
  bool in_forms_section = false;

  RCLCPP_INFO(logger_, "Reading forms from: %s", yaml_path.c_str());

  while (std::getline(file, line)) {
    std::string trimmed = trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    if (trimmed.find("forms:") == 0) {
      in_forms_section = true;
      continue;
    }

    if (in_forms_section) {
      size_t dash_pos = trimmed.find('-');
      if (dash_pos != std::string::npos) {
        std::string form = trim(trimmed.substr(dash_pos + 1));

        if (form.empty() || form[0] == '#') {
          continue;
        }

        if (form.front() == '"' || form.front() == '\'') {
          form = form.substr(1);
        }
        if (form.back() == '"' || form.back() == '\'') {
          form = form.substr(0, form.length() - 1);
        }

        forms.push_back(form);
      } else if (!trimmed.empty() && trimmed[0] != ' ' && trimmed[0] != '-') {
        break;
      }
    }
  }

  file.close();

  RCLCPP_INFO(logger_, "Found %zu forms in YAML file", forms.size());

  for (const auto &form : forms) {
    parseWKT(form, "", map_frame_, static_cast<CostLevel>(default_cost), -1.0);
  }

  RCLCPP_INFO(logger_,
              "Successfully loaded: %zu circles, %zu lines, %zu polygons",
              circles_.size(), lines_.size(), polygons_.size());
}

// =======================
// Coordinate Parser Helper
// =======================
std::vector<std::pair<double, double>>
parseCoordinates(const std::string &coords_str) {
  std::vector<std::pair<double, double>> points;
  std::string cleaned = coords_str;

  std::string::iterator new_end =
      std::unique(cleaned.begin(), cleaned.end(),
                  [](char a, char b) { return a == ' ' && b == ' '; });
  cleaned.erase(new_end, cleaned.end());

  std::istringstream ss(cleaned);
  std::string pair_str;

  while (std::getline(ss, pair_str, ',')) {
    pair_str = trim(pair_str);
    size_t space_pos = pair_str.find(' ');

    if (space_pos != std::string::npos) {
      try {
        std::string x_str = pair_str.substr(0, space_pos);
        std::string y_str = pair_str.substr(space_pos + 1);

        double x = std::stod(trim(x_str));
        double y = std::stod(trim(y_str));
        points.push_back({x, y});
      } catch (const std::exception &e) {
        continue;
      }
    }
  }
  return points;
}

// =======================
// WKT Parser (Enhanced with Duration Support)
// =======================
void VirtualLayer::parseWKT(const std::string &wkt, const std::string &uuid,
                            const std::string &frame, CostLevel cost,
                            double duration) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::string wkt_trimmed = trim(wkt);

  // =====================
  // 1) Defaults
  // =====================
  CostLevel final_cost = cost;
  double final_duration = duration;
  double thickness = 0.3;

  // =====================
  // 2) Extract TAGS (order independent)
  // =====================
  auto extract_tag = [&](const std::string &tag) -> std::optional<std::string> {
    size_t pos = wkt_trimmed.find(tag);
    if (pos == std::string::npos)
      return std::nullopt;

    size_t end = wkt_trimmed.find("]", pos);
    if (end == std::string::npos)
      return std::nullopt;

    return trim(wkt_trimmed.substr(pos + tag.size(), end - pos - tag.size()));
  };

  // COST
  if (auto val = extract_tag("[COST:"); val.has_value()) {
    try {
      final_cost = static_cast<CostLevel>(std::stoi(*val));
    } catch (...) {
    }
  }

  // DURATION
  if (auto val = extract_tag("[DURATION:"); val.has_value()) {
    try {
      final_duration = std::stod(*val);
    } catch (...) {
    }
  }

  // THICKNESS (for lines)
  if (auto val = extract_tag("[THICKNESS:"); val.has_value()) {
    try {
      thickness = std::stod(*val);
    } catch (...) {
    }
  }

  // =====================
  // 3) Remove all tags
  // =====================
  size_t cut_pos = wkt_trimmed.find('[');
  if (cut_pos != std::string::npos) {
    wkt_trimmed = trim(wkt_trimmed.substr(0, cut_pos));
  }

  // Uppercase copy for type detection
  std::string wkt_upper = wkt_trimmed;
  std::transform(wkt_upper.begin(), wkt_upper.end(), wkt_upper.begin(),
                 ::toupper);

  std::string shape_uuid = uuid.empty() ? generateUUID() : uuid;

  // ============================================================
  // CIRCLE (x y r)
  // ============================================================
  if (wkt_upper.find("CIRCLE") == 0) {
    size_t start = wkt_trimmed.find("(");
    size_t end = wkt_trimmed.rfind(")");

    if (start != std::string::npos && end != std::string::npos && end > start) {
      std::istringstream ss(wkt_trimmed.substr(start + 1, end - start - 1));

      Circle c;
      if (ss >> c.x >> c.y >> c.r) {
        c.uuid = shape_uuid;
        c.frame_id = frame;
        c.timestamp = node->now();
        c.cost_level = final_cost;
        c.duration_seconds = final_duration;

        circles_[shape_uuid] = c;

        RCLCPP_INFO(
            logger_,
            "Added CIRCLE [%s]: x=%.2f y=%.2f r=%.2f cost=%d duration=%s",
            shape_uuid.c_str(), c.x, c.y, c.r, static_cast<int>(final_cost),
            final_duration < 0
                ? "infinite"
                : (std::to_string(final_duration) + "s").c_str());
      }
    }
  }

  // ============================================================
  // LINESTRING
  // ============================================================
  else if (wkt_upper.find("LINESTRING") == 0) {
    size_t start = wkt_trimmed.find("(");
    size_t end = wkt_trimmed.rfind(")");

    if (start != std::string::npos && end != std::string::npos && end > start) {
      auto points =
          parseCoordinates(wkt_trimmed.substr(start + 1, end - start - 1));

      if (points.size() >= 2) {
        Line l;
        l.uuid = shape_uuid;
        l.frame_id = frame;
        l.timestamp = node->now();
        l.cost_level = final_cost;
        l.duration_seconds = final_duration;
        l.thickness = thickness;

        l.x1 = points[0].first;
        l.y1 = points[0].second;
        l.x2 = points[1].first;
        l.y2 = points[1].second;

        lines_[shape_uuid] = l;

        RCLCPP_INFO(logger_,
                    "Added LINESTRING [%s]: (%.2f,%.2f)->(%.2f,%.2f) "
                    "thick=%.2f cost=%d duration=%s",
                    shape_uuid.c_str(), l.x1, l.y1, l.x2, l.y2, l.thickness,
                    static_cast<int>(final_cost),
                    final_duration < 0
                        ? "infinite"
                        : (std::to_string(final_duration) + "s").c_str());
      }
    }
  }

  // ============================================================
  // POLYGON
  // ============================================================
  else if (wkt_upper.find("POLYGON") == 0) {
    size_t start = wkt_trimmed.find("(");
    size_t end = wkt_trimmed.rfind(")");

    if (start != std::string::npos && end != std::string::npos && end > start) {
      std::string coords = wkt_trimmed.substr(start + 1, end - start - 1);

      bool is_filled = false;
      if (coords.find("(),") == 0 || coords.find("() ,") == 0) {
        is_filled = true;
        coords = trim(coords.substr(coords.find(',') + 1));
      }

      auto points = parseCoordinates(coords);
      if (points.size() >= 3) {
        Polygon p;
        p.uuid = shape_uuid;
        p.frame_id = frame;
        p.timestamp = node->now();
        p.cost_level = final_cost;
        p.duration_seconds = final_duration;
        p.is_filled = is_filled;

        for (const auto &pt : points) {
          geometry_msgs::msg::Point gp;
          gp.x = pt.first;
          gp.y = pt.second;
          gp.z = 0.0;
          p.points.push_back(gp);
        }

        polygons_[shape_uuid] = p;

        RCLCPP_INFO(logger_,
                    "Added POLYGON [%s]: %zu points %s cost=%d duration=%s",
                    shape_uuid.c_str(), p.points.size(),
                    is_filled ? "filled" : "open", static_cast<int>(final_cost),
                    final_duration < 0
                        ? "infinite"
                        : (std::to_string(final_duration) + "s").c_str());
      }
    }
  }
}

// =======================
// Initialize
// =======================
void VirtualLayer::onInitialize() {
  auto node = node_.lock();

  logger_ = node->get_logger().get_child("virtual_layer");

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("forms", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("forms_file", rclcpp::ParameterValue(""));
  declareParameter("line_thickness", rclcpp::ParameterValue(0.3));
  declareParameter("map_frame", rclcpp::ParameterValue("map"));
  declareParameter("default_cost_level", rclcpp::ParameterValue(254));
  declareParameter("expiration_check_frequency", rclcpp::ParameterValue(1.0));

  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  double default_thickness =
      node->get_parameter(name_ + ".line_thickness").as_double();
  map_frame_ = node->get_parameter(name_ + ".map_frame").as_string();
  int default_cost =
      node->get_parameter(name_ + ".default_cost_level").as_int();
  double check_frequency =
      node->get_parameter(name_ + ".expiration_check_frequency").as_double();

  tf_buffer_ = tf_;

  // Setup expiration timer
  auto timer_period = std::chrono::duration<double>(1.0 / check_frequency);
  expiration_timer_ = node->create_wall_timer(
      timer_period, std::bind(&VirtualLayer::checkExpiredShapes, this));

  std::string forms_file =
      node->get_parameter(name_ + ".forms_file").as_string();

  if (!forms_file.empty()) {
    std::string resolved_path = resolveFormsFilePath(forms_file);

    std::ifstream file_check(resolved_path);
    if (file_check.good()) {
      RCLCPP_INFO(logger_, "Loading forms from external YAML: %s",
                  resolved_path.c_str());
      loadFormsFromYAML(resolved_path);
    } else {
      RCLCPP_ERROR(logger_, "Forms file not found: %s (resolved from: %s)",
                   resolved_path.c_str(), forms_file.c_str());
    }
  } else {
    auto forms = node->get_parameter(name_ + ".forms").as_string_array();

    RCLCPP_INFO(logger_,
                "Parsing %zu WKT forms from parameters in frame '%s' with "
                "default cost %d...",
                forms.size(), map_frame_.c_str(), default_cost);

    for (const auto &form : forms) {
      parseWKT(form, "", map_frame_, static_cast<CostLevel>(default_cost),
               -1.0);
    }
  }

  {
    std::lock_guard<std::mutex> lock(shapes_mutex_);
    for (auto &[uuid, line] : lines_) {
      line.thickness = default_thickness;
    }
  }

  // Setup ROS2 Interfaces
  shape_sub_ = node->create_subscription<std_msgs::msg::String>(
      "virtual_layer/shapes", 10,
      std::bind(&VirtualLayer::shapeCallback, this, std::placeholders::_1));

  add_circle_srv_ = node->create_service<nav2_virtual_layer::srv::AddCircle>(
      "virtual_layer/add_circle",
      std::bind(&VirtualLayer::addCircleCallback, this, std::placeholders::_1,
                std::placeholders::_2));

  add_line_srv_ = node->create_service<nav2_virtual_layer::srv::AddLine>(
      "virtual_layer/add_line",
      std::bind(&VirtualLayer::addLineCallback, this, std::placeholders::_1,
                std::placeholders::_2));

  add_polygon_srv_ = node->create_service<nav2_virtual_layer::srv::AddPolygon>(
      "virtual_layer/add_polygon",
      std::bind(&VirtualLayer::addPolygonCallback, this, std::placeholders::_1,
                std::placeholders::_2));

  remove_shape_srv_ =
      node->create_service<nav2_virtual_layer::srv::RemoveShape>(
          "virtual_layer/remove_shape",
          std::bind(&VirtualLayer::removeShapeCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  clear_all_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "virtual_layer/clear_all",
      std::bind(&VirtualLayer::clearAllCallback, this, std::placeholders::_1,
                std::placeholders::_2));

  restore_shapes_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "virtual_layer/restore_defaults",
      std::bind(&VirtualLayer::restoreDefaultsCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  current_ = true;

  RCLCPP_INFO(logger_,
              "VirtualLayer initialized: %zu circles, %zu lines, %zu polygons",
              circles_.size(), lines_.size(), polygons_.size());
}

// =======================
// Service Callbacks
// =======================
void VirtualLayer::addCircleCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddCircle::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddCircle::Response> response) {

  double final_duration = (request->duration != 0.0) ? request->duration : -1.0;
  std::string uuid =
      addCircle(request->x, request->y, request->radius,
                request->frame_id.empty() ? map_frame_ : request->frame_id,
                static_cast<CostLevel>(request->cost_level), final_duration);

  response->uuid = uuid;
  response->success = !uuid.empty();

  auto node = node_.lock();
  RCLCPP_INFO(logger_, "Service added circle: %s (duration=%s)", uuid.c_str(),
              final_duration < 0 ? "infinite"
                                 : std::to_string(final_duration).c_str());
}

void VirtualLayer::addLineCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddLine::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddLine::Response> response) {
  double final_duration = (request->duration != 0.0) ? request->duration : -1.0;

  std::string uuid = addLine(
      request->x1, request->y1, request->x2, request->y2, request->thickness,
      request->frame_id.empty() ? map_frame_ : request->frame_id,
      static_cast<CostLevel>(request->cost_level), final_duration);

  response->uuid = uuid;
  response->success = !uuid.empty();

  auto node = node_.lock();
  RCLCPP_INFO(logger_, "Service added line: %s (duration=%s)", uuid.c_str(),
              final_duration < 0 ? "infinite"
                                 : std::to_string(final_duration).c_str());
}

void VirtualLayer::addPolygonCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddPolygon::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddPolygon::Response> response) {

  double final_duration = (request->duration != 0.0) ? request->duration : -1.0;

  std::string uuid =
      addPolygon(request->points,
                 request->frame_id.empty() ? map_frame_ : request->frame_id,
                 static_cast<CostLevel>(request->cost_level), final_duration);

  response->uuid = uuid;
  response->success = !uuid.empty();

  auto node = node_.lock();
  RCLCPP_INFO(logger_, "Service added polygon: %s (duration=%s)", uuid.c_str(),
              final_duration < 0 ? "infinite"
                                 : std::to_string(final_duration).c_str());
}

void VirtualLayer::removeShapeCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::RemoveShape::Request>
        request,
    std::shared_ptr<nav2_virtual_layer::srv::RemoveShape::Response> response) {
  response->success = removeShape(request->uuid);

  auto node = node_.lock();
  RCLCPP_INFO(logger_, "Service removed shape: %s (%s)", request->uuid.c_str(),
              response->success ? "success" : "not found");
}

void VirtualLayer::clearAllCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  clearAllShapes();

  response->success = true;
  response->message = "All shapes cleared successfully";

  auto node = node_.lock();
  RCLCPP_INFO(logger_, "%s", response->message.c_str());
}

void VirtualLayer::restoreDefaultsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  auto node = node_.lock();

  clearAllShapes();

  bool success = true;
  std::string msg;

  std::string forms_file =
      node->get_parameter(name_ + ".forms_file").as_string();

  if (!forms_file.empty()) {
    std::string resolved_path = resolveFormsFilePath(forms_file);

    if (!resolved_path.empty()) {
      loadFormsFromYAML(resolved_path);
      msg = "Defaults restored from YAML file";
    } else {
      success = false;
      msg = "Failed to resolve forms file path";
    }
  } else {
    auto forms = node->get_parameter(name_ + ".forms").as_string_array();
    int default_cost =
        node->get_parameter(name_ + ".default_cost_level").as_int();

    for (const auto &form : forms) {
      parseWKT(form, "", map_frame_, static_cast<CostLevel>(default_cost),
               -1.0);
    }
    msg = "Defaults restored from parameters";
  }

  response->success = success;
  response->message = msg;

  RCLCPP_INFO(logger_, "Restore defaults: %s (%s)", msg.c_str(),
              success ? "success" : "failed");
}

// =======================
// Topic Callbacks
// =======================
void VirtualLayer::shapeCallback(const std_msgs::msg::String::SharedPtr msg) {
  parseWKT(msg->data, "", map_frame_, COST_LETHAL, -1.0);
}

// =======================
// Add Shape Methods (Thread-Safe)
// =======================
std::string VirtualLayer::addCircle(double x, double y, double r,
                                    const std::string &frame, CostLevel cost,
                                    double duration) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  Circle c;
  c.uuid = generateUUID();
  c.frame_id = frame;
  c.timestamp = node_.lock()->now();
  c.cost_level = cost;
  c.duration_seconds = duration;
  c.x = x;
  c.y = y;
  c.r = r;

  circles_[c.uuid] = c;

  return c.uuid;
}

std::string VirtualLayer::addLine(double x1, double y1, double x2, double y2,
                                  double thickness, const std::string &frame,
                                  CostLevel cost, double duration) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  Line l;
  l.uuid = generateUUID();
  l.frame_id = frame;
  l.timestamp = node_.lock()->now();
  l.cost_level = cost;
  l.duration_seconds = duration;
  l.x1 = x1;
  l.y1 = y1;
  l.x2 = x2;
  l.y2 = y2;
  l.thickness = thickness;

  lines_[l.uuid] = l;

  return l.uuid;
}

std::string
VirtualLayer::addPolygon(const std::vector<geometry_msgs::msg::Point> &points,
                         const std::string &frame, CostLevel cost,
                         double duration) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  Polygon poly;
  poly.uuid = generateUUID();
  poly.frame_id = frame;
  poly.timestamp = node_.lock()->now();
  poly.cost_level = cost;
  poly.duration_seconds = duration;
  poly.is_filled = false;
  poly.points = points;

  polygons_[poly.uuid] = poly;

  return poly.uuid;
}

// =======================
// Remove Shape (Thread-Safe)
// =======================
bool VirtualLayer::removeShape(const std::string &uuid) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  // Try to remove from each shape container
  if (circles_.erase(uuid) > 0)
    return true;
  if (lines_.erase(uuid) > 0)
    return true;
  if (polygons_.erase(uuid) > 0)
    return true;

  return false;
}

void VirtualLayer::clearAllShapes() {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  circles_.clear();
  lines_.clear();
  polygons_.clear();
}

// =======================
// Get Shape Info
// =======================
size_t VirtualLayer::getShapeCount() const {
  std::lock_guard<std::mutex> lock(shapes_mutex_);
  return circles_.size() + lines_.size() + polygons_.size();
}

std::vector<std::string> VirtualLayer::getAllShapeUUIDs() const {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  std::vector<std::string> uuids;
  for (const auto &[uuid, _] : circles_)
    uuids.push_back(uuid);
  for (const auto &[uuid, _] : lines_)
    uuids.push_back(uuid);
  for (const auto &[uuid, _] : polygons_)
    uuids.push_back(uuid);

  return uuids;
}

// =======================
// Transform Point
// =======================
bool VirtualLayer::transformPoint(double x_in, double y_in,
                                  const std::string &target_frame,
                                  double &x_out, double &y_out) {
  // No transformation needed if frames match
  if (map_frame_ == target_frame) {
    x_out = x_in;
    y_out = y_in;
    return true;
  }

  if (tf_buffer_ == nullptr) {
    auto node = node_.lock();
    RCLCPP_WARN_ONCE(logger_,
                     "TF buffer not available, cannot transform points");
    return false;
  }

  try {
    geometry_msgs::msg::PointStamped point_in, point_out;
    point_in.header.frame_id = map_frame_;
    point_in.header.stamp = rclcpp::Time(0);
    point_in.point.x = x_in;
    point_in.point.y = y_in;
    point_in.point.z = 0.0;

    point_out = tf_buffer_->transform(point_in, target_frame,
                                      tf2::durationFromSec(0.1));

    x_out = point_out.point.x;
    y_out = point_out.point.y;
    return true;
  } catch (tf2::TransformException &ex) {
    auto node = node_.lock();
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 5000,
                         "Could not transform from %s to %s: %s",
                         map_frame_.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

// =======================
// Update Bounds
// =======================
void VirtualLayer::updateBounds(double, double, double, double *min_x,
                                double *min_y, double *max_x, double *max_y) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(shapes_mutex_);

  std::string global_frame = layered_costmap_->getGlobalFrameID();

  // Update bounds for circles
  for (const auto &[uuid, c] : circles_) {
    double cx, cy;
    if (!transformPoint(c.x, c.y, global_frame, cx, cy)) {
      continue;
    }

    *min_x = std::min(*min_x, cx - c.r);
    *min_y = std::min(*min_y, cy - c.r);
    *max_x = std::max(*max_x, cx + c.r);
    *max_y = std::max(*max_y, cy + c.r);
  }

  // Update bounds for lines
  for (const auto &[uuid, l] : lines_) {
    double x1, y1, x2, y2;
    if (!transformPoint(l.x1, l.y1, global_frame, x1, y1) ||
        !transformPoint(l.x2, l.y2, global_frame, x2, y2)) {
      continue;
    }

    double pad = l.thickness;

    *min_x = std::min({*min_x, x1 - pad, x2 - pad});
    *min_y = std::min({*min_y, y1 - pad, y2 - pad});
    *max_x = std::max({*max_x, x1 + pad, x2 + pad});
    *max_y = std::max({*max_y, y1 + pad, y2 + pad});
  }

  // Update bounds for polygons
  for (const auto &[uuid, p] : polygons_) {
    for (const auto &pt : p.points) {
      double px, py;
      if (!transformPoint(pt.x, pt.y, global_frame, px, py)) {
        continue;
      }

      *min_x = std::min(*min_x, px);
      *min_y = std::min(*min_y, py);
      *max_x = std::max(*max_x, px);
      *max_y = std::max(*max_y, py);
    }
  }
}

// =======================
// Update Costs - With Cost Levels
// =======================
void VirtualLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                               int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_)
    return;

  current_ = true;
  std::string global_frame = layered_costmap_->getGlobalFrameID();

  // Iterate through all cells in the update region
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      uint8_t shape_cost = 0;
      if (isInsideRestriction(wx, wy, global_frame, shape_cost)) {

        unsigned char final_cost;

        // Map cost levels to Nav2 costmap values
        if (shape_cost >= COST_LETHAL) {
          final_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else if (shape_cost == COST_HIGH) {
          final_cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        } else {
          final_cost = shape_cost;
        }

        // Take the maximum of current cost and shape cost
        unsigned char current_cost = master_grid.getCost(i, j);
        master_grid.setCost(i, j, std::max(current_cost, final_cost));
      }
    }
  }
}

// =======================
// Point in Polygon Test
// =======================
bool VirtualLayer::pointInPolygon(double x, double y, const Polygon &poly) {
  int n = poly.points.size();
  bool inside = false;

  // Ray casting algorithm
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = poly.points[i].x, yi = poly.points[i].y;
    double xj = poly.points[j].x, yj = poly.points[j].y;

    bool intersect = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi);
    if (intersect)
      inside = !inside;
  }
  return inside;
}

// =======================
// Restriction Check - With Cost Output
// =======================
bool VirtualLayer::isInsideRestriction(double x, double y,
                                       const std::string &frame,
                                       uint8_t &cost_out) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  cost_out = 0;
  uint8_t max_cost = 0;

  // Check circles
  for (const auto &[uuid, c] : circles_) {
    double cx, cy;
    if (!transformPoint(c.x, c.y, frame, cx, cy)) {
      continue;
    }

    double dx = x - cx;
    double dy = y - cy;
    if ((dx * dx + dy * dy) <= (c.r * c.r)) {
      max_cost = std::max(max_cost, static_cast<uint8_t>(c.cost_level));
    }
  }

  // Check lines (point-to-line-segment distance)
  for (const auto &[uuid, l] : lines_) {
    double x1, y1, x2, y2;
    if (!transformPoint(l.x1, l.y1, frame, x1, y1) ||
        !transformPoint(l.x2, l.y2, frame, x2, y2)) {
      continue;
    }

    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;

    if (len_sq < 1e-10)
      continue;

    double param = std::max(0.0, std::min(1.0, dot / len_sq));
    double xx = x1 + param * C;
    double yy = y1 + param * D;
    double dx = x - xx;
    double dy = y - yy;

    if ((dx * dx + dy * dy) <= (l.thickness * l.thickness)) {
      max_cost = std::max(max_cost, static_cast<uint8_t>(l.cost_level));
    }
  }

  // Check polygons
  for (const auto &[uuid, poly] : polygons_) {
    Polygon transformed_poly;
    bool transform_success = true;

    // Transform all polygon points to target frame
    for (const auto &pt : poly.points) {
      double tx, ty;
      if (transformPoint(pt.x, pt.y, frame, tx, ty)) {
        geometry_msgs::msg::Point tpt;
        tpt.x = tx;
        tpt.y = ty;
        tpt.z = 0.0;
        transformed_poly.points.push_back(tpt);
      } else {
        transform_success = false;
        break;
      }
    }

    if (transform_success) {
      if (poly.is_filled) {
        // Filled polygon - check if point is inside
        if (pointInPolygon(x, y, transformed_poly)) {
          max_cost = std::max(max_cost, static_cast<uint8_t>(poly.cost_level));
        }
      } else {
        // Open polygon - treat as polyline (sequence of line segments)
        for (size_t i = 0; i + 1 < transformed_poly.points.size(); ++i) {
          const auto &p1 = transformed_poly.points[i];
          const auto &p2 = transformed_poly.points[i + 1];

          double dx = p2.x - p1.x;
          double dy = p2.y - p1.y;
          double len2 = dx * dx + dy * dy;
          if (len2 < 1e-6)
            continue;

          double t = ((x - p1.x) * dx + (y - p1.y) * dy) / len2;
          t = std::clamp(t, 0.0, 1.0);

          double px = p1.x + t * dx;
          double py = p1.y + t * dy;

          double dist2 = (x - px) * (x - px) + (y - py) * (y - py);
          if (dist2 <= 0.05 * 0.05) { // Default thickness for polygon edges
            max_cost =
                std::max(max_cost, static_cast<uint8_t>(poly.cost_level));
          }
        }
      }
    }
  }

  cost_out = max_cost;
  return max_cost > 0;
}

// =======================
// Reset
// =======================
void VirtualLayer::reset() { current_ = true; }

} // namespace nav2_virtual_layer

// Register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(nav2_virtual_layer::VirtualLayer, nav2_costmap_2d::Layer)