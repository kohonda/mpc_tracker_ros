#ifndef FRENET_SERRET_CONVERTER_H
#define FRENET_SERRET_CONVERTER_H

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <climits>
#include "FrenetCoordinate.hpp"
#include "MPCCourse.hpp"
#include "Pose.hpp"

namespace pathtrack_tools
{
///
/// @class FrenetSerretConverter
/// @brief Convert a position described in global coordinates to the Frenet-Serret coordinate system defined by the
/// reference path.
///

class FrenetSerretConverter
{
    public:
    /**
     * @brief Default constructor
     *
     */
    FrenetSerretConverter();

    /**
     * @brief Constructor with option
     *
     * @param use_previous_pose
     */
    FrenetSerretConverter(const bool use_previous_pose);

    /**
     * @brief Destroy the Frenet Serret Converter object
     *
     */
    ~FrenetSerretConverter();

    /**
     * @brief Convert global position to Frenet-Serret position
     *
     * @param mpc_course vecotr x_ref, y_ref, not used yaw_ref, etc.
     * @param current_pose_g (x_g, y_g, z_g ,roll_g, pitch_g ,yaw_g[rad]): ego vehicle position based on Global
     * coordinate
     * @return current_pose_f (x_f, y_f, yaw_f[rad]): ego vehicle position based on Frenet-Serret coordinate
     */
    FrenetCoordinate global2frenet(const MPCCourse& mpc_course, const Pose& pose_g);

    /**
     * @brief Convert Frenet-Serret position to global position
     *
     * @param mpc_course
     * @param pose_f
     * @return Pose
     */
    Pose frenet2global(const MPCCourse& mpc_course, const FrenetCoordinate& pose_f) const;

    private:
    bool use_previous_pose_                  = true;  //!< @brief True: use previous pose for fast calculation, but the objects to be converted must be identical.
    int previous_nearest_index_              = 0;
    double accumulated_x_f_to_nearest_index_ = 0;    //!< @brief accumulated x_f to the nearest index
    int look_ahead_index_range_              = 100;  //!< @brief look ahead index range, which can be changed depending on the speed

    /**
     * @brief Find nearest neighbor point in full search.
     *
     * @param first_point_index
     * @param last_point_index
     * @param mpc_course
     * @param pose_g
     * @return index of nearest reference point
     */
    int search_nearest_point_index(const int& first_point_index, const int& last_point_index, const MPCCourse& mpc_course, const Pose& pose_g) const;

    /**
     * @brief Extract reference point index of the nearest front of vehicle
     *
     * @param nearest_point_index
     * @param mpc_course
     * @param pose_g
     * @return a index of reference point which is located at the front of vehicle
     */
    int extract_front_point(const int& nearest_point_index, const MPCCourse& mpc_course, const Pose& pose_g) const;

    /**
     * @brief Calculate the projection on the reference path from the nearest waypoint to the vehicle position.
     *
     * @param backward_pt_index
     * @param forwade_pt_index
     * @param reference_path
     * @param current_pose_g
     * @return delta x_f from nearest point
     */
    double delta_xf_from_nearest_pt(const int& nearest_point_index, const int& backward_pt_index, const int& forward_pt_index, const MPCCourse& mpc_course, const Pose& pose_g) const;
};

}  // namespace pathtrack_tools
#endif
