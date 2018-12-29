#ifndef LOAM_MATH_UTILS_H
#define LOAM_MATH_UTILS_H


#include "loam_velodyne/Angle.h"
#include "loam_velodyne/Vector3.h"

#include <cmath>

// Quaterniond RPY2Quaternion(Vector3d rpy)
// {
//   double alpha = rpy(0);
//   double beta = rpy(1);
//   double gamma = rpy(2);

//   Quaterniond q;
//   q.w() = cos(alpha/2)*cos(beta/2)*cos(gamma/2) + sin(alpha/2)*sin(beta/2)*sin(gamma/2);
//   q.x() = sin(alpha/2)*cos(beta/2)*cos(gamma/2) - cos(alpha/2)*sin(beta/2)*sin(gamma/2);
//   q.y() = cos(alpha/2)*sin(beta/2)*cos(gamma/2) + sin(alpha/2)*cos(beta/2)*sin(gamma/2);
//   q.z() = cos(alpha/2)*cos(beta/2)*sin(gamma/2) - sin(alpha/2)*sin(beta/2)*cos(gamma/2);

//   return q;
// }

// void Quaternion2Euler(const Quaterniond& q, double& roll, double& pitch, double& yaw)
// {
//   // roll (x-axis rotation)
//   double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
//   double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
//   roll = atan2(sinr_cosp, cosr_cosp);

//   // pitch (y-axis rotation)
//   double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
//   if (fabs(sinp) >= 1)
//     pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//   else
//     pitch = asin(sinp);

//   // yaw (z-axis rotation)
//   double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
//   double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
//   yaw = atan2(siny_cosp, cosy_cosp);
// }

namespace loam {

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}



/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians)
{
  return (float) (radians * 180.0 / M_PI);
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees)
{
  return (float) (degrees * M_PI / 180.0);
}




/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b)
{
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}



/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b, const float& wb)
{
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}


/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float calcPointDistance(const PointT& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}



/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float calcSquaredPointDistance(const PointT& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}



/** \brief Rotate the given vector by the specified angle around the x-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotX(Vector3& v, const Angle& ang)
{
  float y = v.y();
  v.y() = ang.cos() * y - ang.sin() * v.z();
  v.z() = ang.sin() * y + ang.cos() * v.z();
}

/** \brief Rotate the given point by the specified angle around the x-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotX(PointT& p, const Angle& ang)
{
  float y = p.y;
  p.y = ang.cos() * y - ang.sin() * p.z;
  p.z = ang.sin() * y + ang.cos() * p.z;
}



/** \brief Rotate the given vector by the specified angle around the y-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotY(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x + ang.sin() * v.z();
  v.z() = ang.cos() * v.z() - ang.sin() * x;
}

/** \brief Rotate the given point by the specified angle around the y-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotY(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x + ang.sin() * p.z;
  p.z = ang.cos() * p.z - ang.sin() * x;
}



/** \brief Rotate the given vector by the specified angle around the z-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotZ(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x - ang.sin() * v.y();
  v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given point by the specified angle around the z-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotZ(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x - ang.sin() * p.y;
  p.y = ang.sin() * x + ang.cos() * p.y;
}



/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(v, angZ);
  rotX(v, angX);
  rotY(v, angY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 *
 * @param p the point to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
template <typename PointT>
inline void rotateZXY(PointT& p,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(p, angZ);
  rotX(p, angX);
  rotY(p, angY);
}



/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 *
 * @param v the vector to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(v, angY);
  rotX(v, angX);
  rotZ(v, angZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 *
 * @param p the point to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
template <typename PointT>
inline void rotateYXZ(PointT& p,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(p, angY);
  rotX(p, angX);
  rotZ(p, angZ);
}

} // end namespace loam


#endif // LOAM_MATH_UTILS_H
