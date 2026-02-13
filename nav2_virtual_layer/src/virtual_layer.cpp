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
    : enabled_(true), map_frame_("map"), tf_buffer_(nullptr), next_shape_id_(1) {}

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
// Shape ID Management
// =======================
int VirtualLayer::assignShapeID(const std::string& uuid, int requested_id) {
  int assigned_id;
  
  if (requested_id > 0 && id_to_uuid_.find(requested_id) == id_to_uuid_.end()) {
    // Use requested ID if available
    assigned_id = requested_id;
    if (requested_id >= next_shape_id_) {
      next_shape_id_ = requested_id + 1;
    }
  } else {
    // Auto-assign next available ID
    assigned_id = next_shape_id_++;
  }
  
  id_to_uuid_[assigned_id] = uuid;
  uuid_to_id_[uuid] = assigned_id;
  
  return assigned_id;
}

void VirtualLayer::unregisterShapeID(const std::string& uuid) {
  auto it = uuid_to_id_.find(uuid);
  if (it != uuid_to_id_.end()) {
    int shape_id = it->second;
    id_to_uuid_.erase(shape_id);
    uuid_to_id_.erase(it);
  }
}

std::string VirtualLayer::getUUIDFromID(int shape_id) const {
  auto it = id_to_uuid_.find(shape_id);
  return (it != id_to_uuid_.end()) ? it->second : "";
}

int VirtualLayer::getIDFromUUID(const std::string& uuid) const {
  auto it = uuid_to_id_.find(uuid);
  return (it != uuid_to_id_.end()) ? it->second : -1;
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

  // {uuid, type, id}
  std::vector<std::tuple<std::string, std::string, int>> expired_shapes;

  // =====================
  // Check expired circles
  // =====================
  for (const auto &[uuid, circle] : circles_) {
    if (circle.isExpired(current_time)) {
      expired_shapes.emplace_back(uuid, "CIRCLE", circle.shape_id);
      RCLCPP_DEBUG(logger_, "Circle [ID:%d, UUID:%s] expired after %.1f seconds",
                   circle.shape_id, uuid.c_str(), circle.duration_seconds);
    }
  }

  // =====================
  // Check expired lines
  // =====================
  for (const auto &[uuid, line] : lines_) {
    if (line.isExpired(current_time)) {
      expired_shapes.emplace_back(uuid, "LINE", line.shape_id);
      RCLCPP_DEBUG(logger_, "Line [ID:%d, UUID:%s] expired after %.1f seconds",
                   line.shape_id, uuid.c_str(), line.duration_seconds);
    }
  }

  // =====================
  // Check expired polygons
  // =====================
  for (const auto &[uuid, polygon] : polygons_) {
    if (polygon.isExpired(current_time)) {
      expired_shapes.emplace_back(uuid, "POLYGON", polygon.shape_id);
      RCLCPP_DEBUG(logger_, "Polygon [ID:%d, UUID:%s] expired after %.1f seconds",
                   polygon.shape_id, uuid.c_str(), polygon.duration_seconds);
    }
  }

  // =====================
  // Remove expired shapes
  // =====================
  for (const auto &[uuid, type, shape_id] : expired_shapes) {
    unregisterShapeID(uuid);
    circles_.erase(uuid);
    lines_.erase(uuid);
    polygons_.erase(uuid);

    RCLCPP_INFO(logger_, "Removed expired %s [ID:%d, UUID:%s]", 
                type.c_str(), shape_id, uuid.c_str());
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
    parseWKT(form, "", map_frame_, static_cast<CostLevel>(default_cost), -1.0, -1);
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
// WKT Parser (Enhanced with Duration and ID Support)
// =======================
void VirtualLayer::parseWKT(const std::string &wkt, const std::string &uuid,
                            const std::string &frame, CostLevel cost,
                            double duration, int requested_id) {
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
  int final_id = requested_id;

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

  // ID
  if (auto val = extract_tag("[ID:"); val.has_value()) {
    try {
      final_id = std::stoi(*val);
    } catch (...) {
    }
  }

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
        c.shape_id = assignShapeID(shape_uuid, final_id);
        c.frame_id = frame;
        c.timestamp = node->now();
        c.cost_level = final_cost;
        c.duration_seconds = final_duration;

        circles_[shape_uuid] = c;

        RCLCPP_INFO(
            logger_,
            "Added CIRCLE [UUID:%s] [ID:%d]: x=%.2f y=%.2f r=%.2f cost=%d duration=%s",
            c.uuid.c_str(), c.shape_id, c.x, c.y, c.r, static_cast<int>(final_cost),
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
        l.shape_id = assignShapeID(shape_uuid, final_id);
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
                    "Added LINESTRING [UUID:%s] [ID:%d]: (%.2f,%.2f)->(%.2f,%.2f) "
                    "thick=%.2f cost=%d duration=%s",
                    l.uuid.c_str(), l.shape_id, l.x1, l.y1, l.x2, l.y2, l.thickness,
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
        p.shape_id = assignShapeID(shape_uuid, final_id);
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
                    "Added POLYGON [UUID:%s] [ID:%d]: %zu points %s cost=%d duration=%s",
                    p.uuid.c_str(), p.shape_id, p.points.size(),
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

  // ===== Visualization Parameters =====
  declareParameter("enable_visualization", rclcpp::ParameterValue(true));
  declareParameter("visualization_rate", rclcpp::ParameterValue(2.0));
  declareParameter("text_height", rclcpp::ParameterValue(0.5));
  declareParameter("text_color_r", rclcpp::ParameterValue(0.0));
  declareParameter("text_color_g", rclcpp::ParameterValue(0.0));
  declareParameter("text_color_b", rclcpp::ParameterValue(0.0));
  declareParameter("text_color_a", rclcpp::ParameterValue(1.0));

  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  double default_thickness =
      node->get_parameter(name_ + ".line_thickness").as_double();
  map_frame_ = node->get_parameter(name_ + ".map_frame").as_string();
  int default_cost =
      node->get_parameter(name_ + ".default_cost_level").as_int();
  double check_frequency =
      node->get_parameter(name_ + ".expiration_check_frequency").as_double();

  enable_visualization_ = node->get_parameter(name_ + ".enable_visualization").as_bool();
  double viz_rate = node->get_parameter(name_ + ".visualization_rate").as_double();
  text_height_ = node->get_parameter(name_ + ".text_height").as_double();
  
  text_color_.r = node->get_parameter(name_ + ".text_color_r").as_double();
  text_color_.g = node->get_parameter(name_ + ".text_color_g").as_double();
  text_color_.b = node->get_parameter(name_ + ".text_color_b").as_double();
  text_color_.a = node->get_parameter(name_ + ".text_color_a").as_double();

  tf_buffer_ = tf_;

  // Setup expiration timer
  auto timer_period = std::chrono::duration<double>(1.0 / check_frequency);
  expiration_timer_ = node->create_wall_timer(
      timer_period, std::bind(&VirtualLayer::checkExpiredShapes, this));

  std::string forms_file =
      node->get_parameter(name_ + ".forms_file").as_string();

  if (!forms_file.empty()) {
    current_yaml_path_ = resolveFormsFilePath(forms_file);
    
    std::ifstream file_check(current_yaml_path_);
    if (file_check.good()) {
      RCLCPP_INFO(logger_, "Loading forms from external YAML: %s",
                  current_yaml_path_.c_str());
      loadFormsFromYAML(current_yaml_path_);
    } else {
      RCLCPP_ERROR(logger_, "Forms file not found: %s (resolved from: %s)",
                  current_yaml_path_.c_str(), forms_file.c_str());
      current_yaml_path_.clear();
    }
  }else {
    auto forms = node->get_parameter(name_ + ".forms").as_string_array();

    RCLCPP_INFO(logger_,
                "Parsing %zu WKT forms from parameters in frame '%s' with "
                "default cost %d...",
                forms.size(), map_frame_.c_str(), default_cost);

    for (const auto &form : forms) {
      parseWKT(form, "", map_frame_, static_cast<CostLevel>(default_cost),
               -1.0, -1);
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

  reload_shapes_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "virtual_layer/reload_shapes",
    std::bind(&VirtualLayer::reloadShapesCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  if (enable_visualization_) {
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "virtual_layer/shape_markers", 10);
    
    auto viz_period = std::chrono::duration<double>(1.0 / viz_rate);
    visualization_timer_ = node->create_wall_timer(
        viz_period, std::bind(&VirtualLayer::publishVisualizationMarkers, this));
    
    RCLCPP_INFO(logger_, "Visualization enabled at %.1f Hz", viz_rate);
  }

  current_ = true;

  RCLCPP_INFO(logger_,
              "VirtualLayer initialized: %zu circles, %zu lines, %zu polygons",
              circles_.size(), lines_.size(), polygons_.size());
}

// =======================
// Shape Management Methods
// =======================
std::string VirtualLayer::addCircle(double x, double y, double r,
                                    const std::string& frame,
                                    CostLevel cost,
                                    double duration,
                                    int requested_id) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);
  
  auto node = node_.lock();
  if (!node) {
    return "";
  }

  std::string uuid = generateUUID();
  
  Circle c;
  c.uuid = uuid;
  c.shape_id = assignShapeID(uuid, requested_id);
  c.x = x;
  c.y = y;
  c.r = r;
  c.frame_id = frame;
  c.cost_level = cost;
  c.duration_seconds = duration;
  c.timestamp = node->now();

  circles_[uuid] = c;

  RCLCPP_INFO(logger_,
              "Added Circle [ID:%d]: (%.2f, %.2f) r=%.2f cost=%d duration=%s",
              c.shape_id, x, y, r, static_cast<int>(cost),
              duration < 0 ? "infinite" : (std::to_string(duration) + "s").c_str());

  return uuid;
}

std::string VirtualLayer::addLine(double x1, double y1, double x2, double y2,
                                   double thickness, const std::string& frame,
                                   CostLevel cost, double duration,
                                   int requested_id) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);
  
  auto node = node_.lock();
  if (!node) {
    return "";
  }

  std::string uuid = generateUUID();
  
  Line l;
  l.uuid = uuid;
  l.shape_id = assignShapeID(uuid, requested_id);
  l.x1 = x1;
  l.y1 = y1;
  l.x2 = x2;
  l.y2 = y2;
  l.thickness = thickness;
  l.frame_id = frame;
  l.cost_level = cost;
  l.duration_seconds = duration;
  l.timestamp = node->now();

  lines_[uuid] = l;

  RCLCPP_INFO(logger_,
              "Added Line [ID:%d]: (%.2f,%.2f)->(%.2f,%.2f) thick=%.2f cost=%d duration=%s",
              l.shape_id, x1, y1, x2, y2, thickness, static_cast<int>(cost),
              duration < 0 ? "infinite" : (std::to_string(duration) + "s").c_str());

  return uuid;
}

std::string VirtualLayer::addPolygon(const std::vector<geometry_msgs::msg::Point>& points,
                                     const std::string& frame,
                                     CostLevel cost, double duration,
                                     int requested_id) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);
  
  auto node = node_.lock();
  if (!node) {
    return "";
  }

  std::string uuid = generateUUID();
  
  Polygon p;
  p.uuid = uuid;
  p.shape_id = assignShapeID(uuid, requested_id);
  p.points = points;
  p.frame_id = frame;
  p.cost_level = cost;
  p.duration_seconds = duration;
  p.timestamp = node->now();

  polygons_[uuid] = p;

  RCLCPP_INFO(logger_,
              "Added Polygon [ID:%d]: %zu points cost=%d duration=%s",
              p.shape_id, points.size(), static_cast<int>(cost),
              duration < 0 ? "infinite" : (std::to_string(duration) + "s").c_str());

  return uuid;
}

bool VirtualLayer::removeShape(const std::string& identifier) {
  std::lock_guard<std::mutex> lock(shapes_mutex_);

  std::string uuid = identifier;
  bool removed = false;
  int shape_id = -1;
  std::string shape_type;

  //  Try direct UUID removal first
  auto try_remove_by_uuid = [&](const std::string& id) -> bool {
    if (circles_.erase(id) > 0) {
      shape_type = "Circle";
      shape_id = getIDFromUUID(id);
      return true;
    }
    if (lines_.erase(id) > 0) {
      shape_type = "Line";
      shape_id = getIDFromUUID(id);
      return true;
    }
    if (polygons_.erase(id) > 0) {
      shape_type = "Polygon";
      shape_id = getIDFromUUID(id);
      return true;
    }
    return false;
  };

  removed = try_remove_by_uuid(uuid);

  // If not removed, try as numeric ID
  if (!removed) {
    bool is_number = !identifier.empty() &&
                     std::all_of(identifier.begin(), identifier.end(), ::isdigit);

    if (is_number) {
      int numeric_id = std::stoi(identifier);
      std::string mapped_uuid = getUUIDFromID(numeric_id);

      if (!mapped_uuid.empty()) {
        removed = try_remove_by_uuid(mapped_uuid);
        uuid = mapped_uuid;
        shape_id = numeric_id;
      }
    }
  }


  // Final result
  if (removed) {
    unregisterShapeID(uuid);

    RCLCPP_INFO(logger_,
                "Removed %s [ID:%d, UUID:%s]",
                shape_type.c_str(),
                shape_id,
                uuid.c_str());
  } else {
    RCLCPP_WARN(logger_,
                "No shape found with identifier: %s",
                identifier.c_str());
  }

  return removed;
}


void VirtualLayer::clearAllShapes() {
  std::lock_guard<std::mutex> lock(shapes_mutex_);
  
  size_t total = circles_.size() + lines_.size() + polygons_.size();
  
  circles_.clear();
  lines_.clear();
  polygons_.clear();
  id_to_uuid_.clear();
  uuid_to_id_.clear();
  next_shape_id_ = 1;

  RCLCPP_INFO(logger_, "Cleared all shapes (%zu total)", total);
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
  
  if (response->success) {
    response->shape_id = getIDFromUUID(uuid);
  }
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
  
  if (response->success) {
    response->shape_id = getIDFromUUID(uuid);
  }
}

void VirtualLayer::addPolygonCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddPolygon::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddPolygon::Response> response) {
  
  double final_duration = (request->duration != 0.0) ? request->duration : -1.0;

  std::string uuid = addPolygon(
      request->points,
      request->frame_id.empty() ? map_frame_ : request->frame_id,
      static_cast<CostLevel>(request->cost_level), final_duration);

  response->uuid = uuid;
  response->success = !uuid.empty();
  
  if (response->success) {
    response->shape_id = getIDFromUUID(uuid);
  }
}

void VirtualLayer::removeShapeCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::RemoveShape::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::RemoveShape::Response> response) {
  
  response->success = removeShape(request->identifier);
  
  if (response->success) {
    response->message = "Shape removed successfully";
  } else {
    response->message = "Shape not found: " + request->identifier;
  }
}

void VirtualLayer::clearAllCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  
  clearAllShapes();
  response->success = true;
  response->message = "All shapes cleared";
}

void VirtualLayer::restoreDefaultsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  
  clearAllShapes();
  
  // Reload from YAML or parameters
  auto node = node_.lock();
  if (!node) {
    response->success = false;
    response->message = "Failed to get node";
    return;
  }
  
  std::string forms_file = node->get_parameter(name_ + ".forms_file").as_string();
  
  if (!forms_file.empty()) {
    std::string resolved_path = resolveFormsFilePath(forms_file);
    loadFormsFromYAML(resolved_path);
  }
  
  response->success = true;
  response->message = "Default shapes restored";
}


void VirtualLayer::reloadShapesCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  
  auto node = node_.lock();
  if (!node) {
    response->success = false;
    response->message = "Failed to get node";
    return;
  }

  if (current_yaml_path_.empty()) {
    response->success = false;
    response->message = "No YAML file was loaded initially. Cannot reload.";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
    return;
  }

  std::ifstream file_check(current_yaml_path_);
  if (!file_check.good()) {
    response->success = false;
    response->message = "YAML file not found: " + current_yaml_path_;
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
    return;
  }
  file_check.close();

  RCLCPP_INFO(logger_, "Reloading shapes from YAML: %s", current_yaml_path_.c_str());

  size_t old_count = circles_.size() + lines_.size() + polygons_.size();

  clearAllShapes();

  loadFormsFromYAML(current_yaml_path_);

  size_t new_count = circles_.size() + lines_.size() + polygons_.size();

  response->success = true;
  response->message = "Shapes reloaded successfully from " + current_yaml_path_ + 
                      " (Old: " + std::to_string(old_count) + 
                      " shapes, New: " + std::to_string(new_count) + " shapes)";
  
  RCLCPP_INFO(logger_, "%s", response->message.c_str());
  RCLCPP_INFO(logger_, "Current shapes: %zu circles, %zu lines, %zu polygons",
              circles_.size(), lines_.size(), polygons_.size());
}

void VirtualLayer::shapeCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(logger_, "Received WKT shape: %s", msg->data.c_str());
  
  auto node = node_.lock();
  int default_cost = node->get_parameter(name_ + ".default_cost_level").as_int();
  
  parseWKT(msg->data, "", map_frame_, static_cast<CostLevel>(default_cost), -1.0, -1);
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
// Create Text Marker
// =======================
visualization_msgs::msg::Marker VirtualLayer::createTextMarker(
    int id, const std::string& text, 
    double x, double y, double z,
    const std::string& frame_id) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time();
  marker.ns = "shape_ids";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  
  marker.scale.z = text_height_;  // Text height
  
  marker.color = text_color_;
  
  marker.text = text;
  marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(1000));
  
  return marker;
}

// =======================
// Publish Visualization Markers
// =======================
void VirtualLayer::publishVisualizationMarkers() {
  if (!enable_visualization_ || !marker_pub_) {
    return;
  }

  std::lock_guard<std::mutex> lock(shapes_mutex_);
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  // =====================
  // Circles
  // =====================
  for (const auto &[uuid, circle] : circles_) {
    double cx = circle.x;
    double cy = circle.y;
    double z = 0.5;  // Height above ground for smooth visibility
    
    std::string text = "ID:" + std::to_string(circle.shape_id);
    auto marker = createTextMarker(circle.shape_id, text, cx, cy, z, circle.frame_id);
    marker_array.markers.push_back(marker);
  }
  
  // =====================
  // Lines - Show ID at midpoint
  // =====================
  for (const auto &[uuid, line] : lines_) {
    double mx = (line.x1 + line.x2) / 2.0;
    double my = (line.y1 + line.y2) / 2.0;
    double z = 0.5;
    
    std::string text = "ID:" + std::to_string(line.shape_id);
    auto marker = createTextMarker(
        line.shape_id + 10000,  // Offset to avoid ID collision
        text, mx, my, z, line.frame_id);
    marker_array.markers.push_back(marker);
  }
  
  // =====================
  // Polygons - Show ID at centroid
  // =====================
  for (const auto &[uuid, poly] : polygons_) {
    if (poly.points.empty()) continue;
    
    // Calculate centroid
    double cx = 0.0, cy = 0.0;
    for (const auto &pt : poly.points) {
      cx += pt.x;
      cy += pt.y;
    }
    cx /= poly.points.size();
    cy /= poly.points.size();
    double z = 0.5;
    
    std::string text = "ID:" + std::to_string(poly.shape_id);
    auto marker = createTextMarker(
        poly.shape_id + 20000,  // Offset to avoid ID collision
        text, cx, cy, z, poly.frame_id);
    marker_array.markers.push_back(marker);
  }
  
  // =====================
  // Publish all markers
  // =====================
  if (!marker_array.markers.empty()) {
    marker_pub_->publish(marker_array);
  }
}

// =======================
// Reset
// =======================
void VirtualLayer::reset() { current_ = true; }

} // namespace nav2_virtual_layer

// Register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(nav2_virtual_layer::VirtualLayer, nav2_costmap_2d::Layer)