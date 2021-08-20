#pragma once
#include <iostream>
#include <vector>

///
/// @struct Reference Course for MPC
/// @brief Date interface of reference path, speed, drivable are
///
struct MPCCourse
{
public:
    // NOTE : ALL VECTORs MUST HAVE THE SAME SIZE.
    std::vector<double> x;                       //!< @brief reference position x vector, global coordinate
    std::vector<double> y;                       //!< @brief reference position y vector, global coordinate
    std::vector<double> z;                       //!< @brief reference position z vector, global coordinate, now only used for globa2frenet
    std::vector<double> yaw;                     //!< @brief yaw pose yaw[rad] vector, now not used
    std::vector<double> speed;                   //!< @brief speed[m/s] vector
    std::vector<double> curvature;               //!< @brief curvature vector
    std::vector<double> accumulated_path_length; //!< @brief Int egral of the reference path, which is x_f_ref[m]
    std::vector<double> drivable_width;          //!< @brief drivable cource width[m]

    /**
     * @brief clear all vectors
     *
     */
    void clear();

    /**
     * @brief check size of all vectors
     *
     * @return size, or -1 if the size of all vecttors are not same
     */
    int size() const;

    /**
     * @brief initialize resize of zero vector with default constructor
     *
     * @param vector size
     */
    void resize(const int &size);
};

inline void MPCCourse::clear()
{
    x.clear();
    y.clear();
    z.clear();
    yaw.clear();
    speed.clear();
    curvature.clear();
    accumulated_path_length.clear();
    drivable_width.clear();
}

inline int MPCCourse::size() const
{
    if (x.size() == y.size() && x.size() == z.size() && x.size() == yaw.size() && x.size() == speed.size() && x.size() == curvature.size() && x.size() == accumulated_path_length.size() &&
        x.size() == drivable_width.size())
    {
        return x.size();
    }
    else
    {
        std::cerr << "[MPCCourse] Reference Cource size is invalid!" << std::endl;
        return -1;
    }
}

inline void MPCCourse::resize(const int &size)
{
    x.resize(size);
    y.resize(size);
    z.resize(size);
    yaw.resize(size);
    speed.resize(size);
    curvature.resize(size);
    accumulated_path_length.resize(size);
    drivable_width.resize(size);
}