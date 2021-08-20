#pragma once

#include <Eigen/Dense>

///
/// @struct Vechicle 2D pose in frenet-serret coordinate system
///
struct FrenetCoordinate
{
    double x_f;  //!< @brief vehicle positon x[m] in frenet-serret coordinate, which means the distance traveled along
                 //!< the reference path
    double y_f;  //!< @brief vehicle positon y[m] in frenet-serret coordinate, which means the distance of the vertical
                 //!< line drawn from the reference path
    double yaw_f;  //!< @brief vehicle positon yaw angle[rad] in frenet-serret coordinate, which means yaw_g - yaw_ref
                   //!< (yaw_g: vehicle yaw angle in global coordinate, yaw_ref: reference path yaw angle in global
                   //!< coordinate)

    FrenetCoordinate() : x_f(0), y_f(0), yaw_f(0)
    {
    }
    FrenetCoordinate(const double& x_f, const double& y_f, const double& yaw_f) : x_f(x_f), y_f(y_f), yaw_f(yaw_f)
    {
    }

    Eigen::Vector3d xyyaw() const;
};

inline Eigen::Vector3d FrenetCoordinate::xyyaw() const
{
    return Eigen::Vector3d(x_f, y_f, yaw_f);
}
