#pragma once


#include <cgv/math/quat.h>
#include <cgv/math/fvec.h>
#include <cgv/render/render_types.h>
#include <cmath>


template <typename T>
inline cgv::render::render_types::quat to_quat(const T& roll, const T& pitch, const T& yaw)
{
    T cy = cos(yaw * 0.5);
    T sy = sin(yaw * 0.5);
    T cp = cos(pitch * 0.5);
    T sp = sin(pitch * 0.5);
    T cr = cos(roll * 0.5);
    T sr = sin(roll * 0.5);

    cgv::render::render_types::quat q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

template <typename T>
inline cgv::render::render_types::quat euler_angels_to_quat(const cgv::math::fvec<T,3>& euler)
{
    return to_quat(euler.x(),euler.y(),euler.z());
}




inline cgv::render::render_types::vec3 to_euler_angels(const cgv::render::render_types::dquat& q) {
    cgv::render::render_types::vec3 angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.x() = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1.0)
        angles.y() = std::copysign(M_PI / 2.0, sinp);
    else
        angles.y() = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    angles.z() = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}
