#pragma once

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <array>
#include <limits>
#include <Eigen/Dense>
#include <unordered_map>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include "rapidcsv.h"
#include "MPCCourse.hpp"
#include "Pose.hpp"

namespace pathtrack_tools
{
   ///
   /// @class CourseManager
   /// @brief Manage ego car reference path (x,y,z,yaw), reference speed (speed) and drivable area (delta_x_e<-[now not
   /// supported], delta_y_e). Their info are decided by user setting (csv format) or upper class planner.
   ///

   class CourseManager
   {
   public:
      /**
     * @brief Default constructor
     *
     */
      CourseManager(/* args */);

      /**
     * @brief Destroy the CourseManager object
     *
     */
      ~CourseManager();

      /**
     * @brief Get the reference path size
     *
     * @return int
     */
      int get_path_size() const;

      /**
     * @brief Get the mpc cource object for test
     *
     * @return MPCCourse
     */
      MPCCourse get_mpc_course() const;

      /**
     * @brief Set the course info from csv file and calculate curvature of path
     *
     * @param csv_file_path
     */
      void set_course_from_csv(const std::string &csv_path);

      /**
       * @brief Set the course from nav msgs path
       * 
       * @param path 
       */
      void set_course_from_nav_msgs(const nav_msgs::Path &path, const double &reference_speed);

      /**
     * @brief Get curvature of the reference path in pose_x_f with interpolation．
     *
     * @param pose_x_f : ego vehicle x in frenet-serret coordinate
     * @return curvature of reference path
     */
      double get_curvature(const double &pose_x_f);

      /**
     * @brief Get reference speed in pose_x_f with interpolation
     *
     * @param pose_x_f : ego vehicle x in frenet-serret coordinate
     * @return reference speed[m/s]
     */
      double get_speed(const double &pose_x_f);

      /**
     * @brief Get drivable course width in pose_x_f
     *
     * @param pose_x_f : ego vehicle x in frenet-serret coordinate
     * @return drivable width, which is often course width[m]
     */
      double get_drivable_width(const double &pose_x_f);

   private:
      MPCCourse mpc_course_; //!< @brief driving course interface using in MPC
      // Parameters for path smoothing and filtering
      const int curvature_smoothing_num_ = 10;       //!< @brief Smoothing value for curvature calculation
      const double max_curvature_change_rate_ = 1.0; //!< @brief Saturate value for curvature change rate [1/m^2]
      const double speed_reduction_rate_ = 0.1;      //!< @brief Reduce the speed reference based on the rate of curvature change; v_ref' = v_ref * exp (-speed_reduction_rate * curvature_rate^2)

      // Parameters for lookup table (xf) -> (nearest index)
      std::unordered_map<double, int> hash_xf2index_; // Hash that connects x_f to the nearest index
      const double hash_resolution_ = 0.01;           // resolution of x_f [m]
      const double redundant_xf_ = 5.0;               // Reserve the lookup table for the extra -redundant_xf[m] from the initial value of mpc_course.

      // variables for linear interpolate
      int nearest_index_;           // Nearest waypoint index
      double nearest_ratio_;        // The internal fraction of nearest_index.
      int second_nearest_index_;    // second nearest reference pointのindex
      double second_nearest_ratio_; // The internal fraction of second nearest_index
      double current_pose_x_f_;     // Position of vehicle in the same prediction step

      void set_accumulated_path_length(const double &offset, MPCCourse *mpc_course);

      std::vector<double> calc_accumulated_path_length(const double &offset, const MPCCourse &mpc_course) const;

      void set_path_curvature(const int &smoothing_num, MPCCourse *mpc_course);

      void set_path_yaw(MPCCourse *mpc_course);

      std::vector<double> calc_path_curvature(const int &smoothing_num, const MPCCourse &mpc_course) const;

      double calc_distance(const std::array<double, 2> &p1, const std::array<double, 2> &p2) const;

      void filtering_based_curvature_rate(const std::vector<double> &accumulated_path_length, std::vector<double> *path_curvature, std::vector<double> *ref_speed);

      void set_hash_xf2index(const MPCCourse &mpc_course, std::unordered_map<double, int> *hash_xf2index);

      double round_resolution(const double &raw_x_f) const;

      inline int search_nearest_index(const MPCCourse &mpc_course, const double &pose_x_f, const int &last_nearest_index) const noexcept;

      int search_nearest_index(const MPCCourse &mpc_course, const Pose &pose) const; // Not used now

      inline int lookup_xf_to_nearest_index(const std::unordered_map<double, int> &hash_xf2index, const double &pose_x_f) const;

      void set_internal_ratio(const MPCCourse &mpc_course, const double &pose_x_f, const int &nearest_index);

      double linear_interpolate(const std::vector<double> &reference_vec, const int &nearest_index, const double &nearest_ratio, const int &second_nearest_index,
                                const double &second_nearest_ratio) const noexcept;
   };

} // namespace pathtrack_tools
