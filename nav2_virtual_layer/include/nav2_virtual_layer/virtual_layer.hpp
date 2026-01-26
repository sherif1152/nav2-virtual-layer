/*********************************************************************
 * Copyright (C) Sherif Fathey - All Rights Reserved
 * 
 * This file is part of the nav2_virtual_layer package.
 * 
 * @file virtual_layer.hpp
 * @brief Virtual costmap layer for Nav2 with dynamic shape management
 * @author Sherif Fathey
 *********************************************************************/

#pragma once

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

// For dynamic updates
#include "std_msgs/msg/string.hpp"

// Custom service messages
#include "nav2_virtual_layer/srv/add_circle.hpp"
#include "nav2_virtual_layer/srv/add_line.hpp"
#include "nav2_virtual_layer/srv/add_polygon.hpp"
#include "nav2_virtual_layer/srv/remove_shape.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "pluginlib/class_list_macros.hpp"


namespace nav2_virtual_layer
{

/**
 * @brief Cost level enumeration for different obstacle severities
 * 
 * Defines the cost values that can be assigned to virtual shapes.
 * Higher values indicate more severe obstacles.
 */
enum CostLevel : uint8_t {
  COST_LETHAL = 254,   // Complete obstacle - robot cannot pass
  COST_HIGH = 253,     // High cost - robot strongly discouraged
  COST_MEDIUM = 200,   // Medium cost - robot prefers to avoid
  COST_LOW = 50,       // Low cost - slight preference to avoid
};

/**
 * @brief Virtual layer plugin for Nav2 costmap
 * 
 * This layer allows dynamic creation and management of virtual obstacles
 * in the costmap using geometric shapes (circles, lines, polygons).
 * Shapes can be defined via WKT strings, services, or YAML files.
 */
class VirtualLayer : public nav2_costmap_2d::Layer
{
public:
  /**
   * @brief Constructor
   */
  VirtualLayer();

  /**
   * @brief Initialize the layer
   * 
   * Called once when the layer is created. Sets up parameters,
   * loads initial shapes, and creates service interfaces.
   */
  void onInitialize() override;

  /**
   * @brief Update the bounds of the costmap affected by this layer
   * 
   * @param robot_x Robot's current x position
   * @param robot_y Robot's current y position
   * @param robot_yaw Robot's current yaw angle
   * @param min_x Minimum x bound to update
   * @param min_y Minimum y bound to update
   * @param max_x Maximum x bound to update
   * @param max_y Maximum y bound to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double *min_x, double *min_y,
    double *max_x, double *max_y) override;

  /**
   * @brief Update the costs in the master costmap
   * 
   * Applies the cost values from virtual shapes to the master costmap
   * within the specified bounds.
   * 
   * @param master_grid The master costmap to update
   * @param min_i Minimum i index to update
   * @param min_j Minimum j index to update
   * @param max_i Maximum i index to update
   * @param max_j Maximum j index to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;

  /**
   * @brief Check if this layer can be cleared
   * @return false - virtual obstacles are persistent
   */
  bool isClearable() override { return false; }
  
  /**
   * @brief Reset the layer to its initial state
   */
  void reset() override;

private:
  bool enabled_;              ///< Whether the layer is enabled
  std::string map_frame_;     ///< Reference frame for shapes
  tf2_ros::Buffer* tf_buffer_; ///< TF buffer for coordinate transformations
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_virtual_layer")}; ///< Logger for the layer


  // ===== Thread Safety =====
  mutable std::mutex shapes_mutex_;  ///< Mutex for thread-safe shape access

  // ===== Shape Structures =====
  
  /**
   * @brief Base structure for all shapes
   * 
   * Contains common properties shared by all geometric shapes.
   */
  struct ShapeBase {
    std::string uuid;         ///< Unique identifier
    std::string frame_id;     ///< Reference frame
    rclcpp::Time timestamp;   ///< Time of creation/addition
    CostLevel cost_level;     ///< Cost level of the shape
    double duration_seconds;  ///< Duration in seconds (-1.0 = infinite)
    
    ShapeBase() 
      : frame_id("map"), 
        cost_level(COST_LETHAL),
        duration_seconds(-1.0) {}
    
    /**
     * @brief Check if shape has expired
     * @param current_time Current ROS time
     * @return true if shape should be removed
     */
    bool isExpired(const rclcpp::Time& current_time) const {
      if (duration_seconds <= 0.0) {
        return false; // Infinite duration
      }
      double elapsed = (current_time - timestamp).seconds();
      return elapsed >= duration_seconds;
    }
  };

  /**
   * @brief Polygon shape structure
   * 
   * Represents a polygonal area that can be filled or just an outline.
   */
  struct Polygon : public ShapeBase {
    std::vector<geometry_msgs::msg::Point> points; ///< Vertices of the polygon
    bool is_filled = false;  ///< Whether the polygon interior is filled
  };

  /**
   * @brief Circle shape structure
   * 
   * Represents a circular obstacle or cost area.
   */
  struct Circle : public ShapeBase {
    double x, y, r;  ///< Center coordinates (x, y) and radius (r)
  };

  /**
   * @brief Line shape structure
   * 
   * Represents a line segment with thickness.
   */
  struct Line : public ShapeBase {
    double x1, y1, x2, y2; ///< Start point (x1, y1) and end point (x2, y2)
    double thickness;       ///< Thickness of the line
  };

  // ===== Shape Storage =====
  std::unordered_map<std::string, Polygon> polygons_; ///< Stored polygons by UUID
  std::unordered_map<std::string, Circle> circles_;   ///< Stored circles by UUID
  std::unordered_map<std::string, Line> lines_;       ///< Stored lines by UUID

  // ===== Timer for Expiration Checking =====
  rclcpp::TimerBase::SharedPtr expiration_timer_; ///< Timer to check for expired shapes

  // ===== ROS2 Communication Interfaces =====
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shape_sub_; ///< Shape WKT subscriber
  
  // Service servers
  rclcpp::Service<nav2_virtual_layer::srv::AddCircle>::SharedPtr add_circle_srv_;
  rclcpp::Service<nav2_virtual_layer::srv::AddLine>::SharedPtr add_line_srv_;
  rclcpp::Service<nav2_virtual_layer::srv::AddPolygon>::SharedPtr add_polygon_srv_;
  rclcpp::Service<nav2_virtual_layer::srv::RemoveShape>::SharedPtr remove_shape_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_all_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_shapes_srv_;

  // ===== Service Callbacks =====
  
  /**
   * @brief Service callback to add a circle
   */
  void addCircleCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddCircle::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddCircle::Response> response);
  
  /**
   * @brief Service callback to add a line
   */
  void addLineCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddLine::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddLine::Response> response);
  
  /**
   * @brief Service callback to add a polygon
   */
  void addPolygonCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::AddPolygon::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::AddPolygon::Response> response);
  
  /**
   * @brief Service callback to remove a shape by UUID
   */
  void removeShapeCallback(
    const std::shared_ptr<nav2_virtual_layer::srv::RemoveShape::Request> request,
    std::shared_ptr<nav2_virtual_layer::srv::RemoveShape::Response> response);
  
  /**
   * @brief Service callback to clear all shapes
   */
  void clearAllCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service callback to restore default shapes
   */
  void restoreDefaultsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ===== Topic Callbacks =====
  
  /**
   * @brief Callback for WKT shape strings
   * @param msg Message containing WKT-formatted shape string
   */
  void shapeCallback(const std_msgs::msg::String::SharedPtr msg);

  // ===== Timer Callbacks =====
  
  /**
   * @brief Check and remove expired shapes
   */
  void checkExpiredShapes();

  // ===== Helper Methods =====
  
  /**
   * @brief Parse Well-Known Text (WKT) string to create shapes
   * 
   * Supports the following formats:
   * - CIRCLE(x y radius) [COST:value] [DURATION:seconds]
   * - LINESTRING(x1 y1, x2 y2) [THICKNESS:value] [COST:value] [DURATION:seconds]
   * - POLYGON(x1 y1, x2 y2, ...) [COST:value] [DURATION:seconds] - open polygon
   * - POLYGON((), x1 y1, x2 y2, ...) [COST:value] [DURATION:seconds] - filled polygon
   * 
   * @param wkt WKT-formatted string
   * @param uuid Optional UUID (generated if empty)
   * @param frame Reference frame for the shape
   * @param cost Default cost level if not specified in WKT
   * @param duration Duration in seconds (-1.0 = infinite)
   */
  void parseWKT(const std::string& wkt, const std::string& uuid = "", 
                const std::string& frame = "map", CostLevel cost = COST_LETHAL,
                double duration = -1.0);
  
  /**
   * @brief Generate a unique identifier (UUID v4)
   * @return UUID string
   */
  std::string generateUUID();

  // ===== Shape Management Methods =====
  
  /**
   * @brief Resolve forms file path from various formats
   * 
   * Supports absolute paths, package:// URIs, and relative paths.
   * 
   * @param forms_file Path specification
   * @return Resolved absolute path
   */
  std::string resolveFormsFilePath(const std::string& forms_file);
  
  /**
   * @brief Load shapes from external YAML file
   * @param yaml_path Path to YAML file
   */
  void loadFormsFromYAML(const std::string& yaml_path);
  
  /**
   * @brief Add a circle shape programmatically
   * @param x Center x coordinate
   * @param y Center y coordinate
   * @param r Radius
   * @param frame Reference frame
   * @param cost Cost level
   * @param duration Duration in seconds (-1.0 = infinite)
   * @return UUID of created shape
   */
  std::string addCircle(double x, double y, double r, 
                        const std::string& frame = "map",
                        CostLevel cost = COST_LETHAL,
                        double duration = -1.0);
  
  /**
   * @brief Add a line shape programmatically
   * @param x1 Start x coordinate
   * @param y1 Start y coordinate
   * @param x2 End x coordinate
   * @param y2 End y coordinate
   * @param thickness Line thickness
   * @param frame Reference frame
   * @param cost Cost level
   * @param duration Duration in seconds (-1.0 = infinite)
   * @return UUID of created shape
   */
  std::string addLine(double x1, double y1, double x2, double y2, double thickness,
                      const std::string& frame = "map",
                      CostLevel cost = COST_LETHAL,
                      double duration = -1.0);
  
  /**
   * @brief Add a polygon shape programmatically
   * @param points Vertices of the polygon
   * @param frame Reference frame
   * @param cost Cost level
   * @param duration Duration in seconds (-1.0 = infinite)
   * @return UUID of created shape
   */
  std::string addPolygon(const std::vector<geometry_msgs::msg::Point>& points,
                         const std::string& frame = "map",
                         CostLevel cost = COST_LETHAL,
                         double duration = -1.0);
  
  /**
   * @brief Remove a shape by UUID
   * @param uuid Shape identifier
   * @return true if shape was found and removed
   */
  bool removeShape(const std::string& uuid);
  
  /**
   * @brief Clear all shapes from the layer
   */
  void clearAllShapes();
  
  /**
   * @brief Get total number of shapes
   * @return Count of all shapes (circles + lines + polygons)
   */
  size_t getShapeCount() const;
  
  /**
   * @brief Get all shape UUIDs
   * @return Vector of all shape identifiers
   */
  std::vector<std::string> getAllShapeUUIDs() const;
  
  /**
   * @brief Transform a point between coordinate frames
   * @param x_in Input x coordinate
   * @param y_in Input y coordinate
   * @param target_frame Target coordinate frame
   * @param x_out Output x coordinate
   * @param y_out Output y coordinate
   * @return true if transformation succeeded
   */
  bool transformPoint(double x_in, double y_in, const std::string& target_frame,
                      double& x_out, double& y_out);
  
  /**
   * @brief Check if a point is inside any restriction zone
   * @param wx World x coordinate
   * @param wy World y coordinate
   * @param frame Coordinate frame
   * @param cost_out Output cost value at this point
   * @return true if point is inside a restriction
   */
  bool isInsideRestriction(double wx, double wy, const std::string& frame,
                           uint8_t& cost_out);
  
  /**
   * @brief Point-in-polygon test using ray casting
   * @param x Point x coordinate
   * @param y Point y coordinate
   * @param poly Polygon to test against
   * @return true if point is inside polygon
   */
  bool pointInPolygon(double x, double y, const Polygon &poly);
  
};

}  // namespace nav2_virtual_layer