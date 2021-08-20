#pragma once
#include <Eigen/Dense>

///
/// @struct Vechicle 3D pose in rectangular coordinate system
///
struct Pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    Pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
    {
    }
    Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0)
    {
    }

    inline Pose operator=(const Pose& p)
    {
        this->x     = p.x;
        this->y     = p.y;
        this->z     = p.z;
        this->roll  = p.roll;
        this->pitch = p.pitch;
        this->yaw   = p.yaw;
        return *this;
    }

    inline Pose operator-(const Pose& p) const
    {
        Pose diff;
        diff.x     = this->x - p.x;
        diff.y     = this->y - p.y;
        diff.z     = this->z - p.z;
        diff.roll  = this->roll - p.roll;
        diff.pitch = this->pitch - p.pitch;
        diff.yaw   = this->yaw - p.yaw;
        return diff;
    }

    inline Pose operator+(const Pose& p) const
    {
        Pose diff;
        diff.x     = this->x + p.x;
        diff.y     = this->y + p.y;
        diff.z     = this->z + p.z;
        diff.roll  = this->roll + p.roll;
        diff.pitch = this->pitch + p.pitch;
        diff.yaw   = this->yaw + p.yaw;
        return diff;
    }

    inline Pose operator+=(const Pose& p)
    {
        this->x += p.x;
        this->y += p.y;
        this->z += p.z;
        this->roll += p.roll;
        this->pitch += p.pitch;
        this->yaw += p.yaw;
        return *this;
    }

    inline Pose operator*(double d) const
    {
        Pose diff;
        diff.x     = this->x * d;
        diff.y     = this->y * d;
        diff.z     = this->z * d;
        diff.roll  = this->roll * d;
        diff.pitch = this->pitch * d;
        diff.yaw   = this->yaw * d;
        return diff;
    }

    inline Pose operator/(double d) const
    {
        Pose diff;
        diff.x     = this->x / d;
        diff.y     = this->y / d;
        diff.z     = this->z / d;
        diff.roll  = this->roll / d;
        diff.pitch = this->pitch / d;
        diff.yaw   = this->yaw / d;
        return diff;
    }

    inline Pose operator/=(double d)
    {
        this->x /= d;
        this->y /= d;
        this->z /= d;
        this->roll /= d;
        this->pitch /= d;
        this->yaw /= d;
        return *this;
    }

    inline bool operator==(const Pose& p)
    {
        bool is_equal = true;
        is_equal      = is_equal && (this->x == p.x);
        is_equal      = is_equal && (this->y == p.y);
        is_equal      = is_equal && (this->z == p.z);
        is_equal      = is_equal && (this->roll == p.roll);
        is_equal      = is_equal && (this->pitch == p.pitch);
        is_equal      = is_equal && (this->yaw == p.yaw);
        return is_equal;
    }

    Eigen::Vector3d xyz() const;
    Eigen::Vector2d xy() const;
    Eigen::Vector3d rpy() const;
    Eigen::Vector3d xyyaw() const;
};

inline Eigen::Vector3d Pose::xyz() const
{
    return Eigen::Vector3d(x, y, z);
}

inline Eigen::Vector2d Pose::xy() const
{
    return Eigen::Vector2d(x, y);
}

inline Eigen::Vector3d Pose::rpy() const
{
    return Eigen::Vector3d(roll, pitch, yaw);
}

inline Eigen::Vector3d Pose::xyyaw() const
{
    return Eigen::Vector3d(x, y, yaw);
}
