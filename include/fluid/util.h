/**
 * @file util.h
 */

#ifndef UTIL_H
#define UTIL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <mavros_msgs/PositionTarget.h>

#include <vector>

/**
 * @brief Holds a bunch of convenience functions.
 */
class Util {
   public:
    
    /**
     * @param n Number you want to square
     *
     * @return The quare of @p n.
     */
    static double sq(double n) {
        return n*n;
    }

    /**
     * @brief If a number is negative, then we return the opposite of
     * the quared root of the absolut value of that number.
     *
     * @param nb Number you want to take the signed squared root
     *
     * @return The quare of @p n.
     */
    static double signed_sqrt(double nb){
    return nb>0 ? sqrt(nb) : -sqrt(-nb);
    }

    /**
     * @param current First point.
     * @param target Second point.
     *
     * @return Eucledian distance betweeen @p current and @p target.
     */
    static double distanceBetween(const geometry_msgs::Point& current, const geometry_msgs::Point& target) {
        double delta_x = target.x - current.x;
        double delta_y = target.y - current.y;
        double delta_z = target.z - current.z;

        return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    }

    /**
     * @brief Creates a path between @p first and @p last which consist of a series of points between them specified
     *        by @p density.
     *
     * @param first The first point.
     * @param last The last point.
     * @param density The density of points between @p first and @p last.
     *
     * @return The new path with inserted points.
     */
    static std::vector<geometry_msgs::Point> createPath(const geometry_msgs::Point& first,
                                                        const geometry_msgs::Point& last, const double& density) {
        double distance = distanceBetween(first, last);

        std::vector<geometry_msgs::Point> path;

        float delta_x = last.x - first.x;
        float delta_y = last.y - first.y;
        float delta_z = last.z - first.z;

        for (int i = 0; i < int(density * distance) + 1; i++) {
            geometry_msgs::Point temp;

            temp.x = first.x + i * delta_x / (density * distance);
            temp.y = first.y + i * delta_y / (density * distance);
            temp.z = first.z + i * delta_z / (density * distance);
            path.insert(path.end(), temp);
        }

        return path;
    }

    /**
     * @brief Sum positition, velocity and acceleration value from two PositionTarget.
     * Remark: the returned PositionTarget will have the same header as the first parameter
     * 
     * @param a The first position target to sum and from which will be taken the header
     * @param b The second position target to sum
     * @return mavros_msgs::PositionTarget 
     */
    static mavros_msgs::PositionTarget addPositionTarget(mavros_msgs::PositionTarget a, mavros_msgs::PositionTarget b){
        mavros_msgs::PositionTarget res;
        res.header = a.header; // this is arbitrary. Did no find a perfect solution, but should not have any impact

        res.position.x = a.position.x + b.position.x;
        res.position.y = a.position.y + b.position.y;
        res.position.z = a.position.z + b.position.z;

        res.velocity.x = a.velocity.x + b.velocity.x;
        res.velocity.y = a.velocity.y + b.velocity.y;
        res.velocity.z = a.velocity.z + b.velocity.z;

        res.acceleration_or_force.x = a.acceleration_or_force.x + b.acceleration_or_force.x;
        res.acceleration_or_force.y = a.acceleration_or_force.y + b.acceleration_or_force.y;
        res.acceleration_or_force.z = a.acceleration_or_force.z + b.acceleration_or_force.z;

        return res;
    }

    /**
     * @brief translate euler angle to Quaternion
     * 
     * @param yaw The euler yaw
     * @param roll The euler roll
     * @param pitch The euler pitch
     * @return The equivalent quaternion 
     */
    static geometry_msgs::Quaternion euler_to_quaternion(double yaw, double roll, double pitch){
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        geometry_msgs::Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    /**
     * @brief translate euler angle to Quaternion
     * 
     * @param euler The euler angle we want to translate. x = pitch, y = roll, z = yaw
     * @return The equivalent quaternion
     */
    static geometry_msgs::Quaternion euler_to_quaternion(geometry_msgs::Vector3 euler){
        return euler_to_quaternion(euler.z, euler.y, euler.x);
    }

    /**
     * @brief Transform an a quaternion orientation into euler angles.
     * 
     * @param orientation The quaternion orientation.
     * 
     * @return The equivalent euler angle.
     */
    static geometry_msgs::Vector3 quaternion_to_euler_angle(geometry_msgs::Quaternion orientation){
        float w = orientation.w;
        float x = orientation.x;
        float y = orientation.y;
        float z = orientation.z;

        geometry_msgs::Vector3 ret;
        float t0 = +2.0 * (w * x + y * z);
        float t1 = +1.0 - 2.0 * (x * x + y * y);
        ret.x = atan2(t0, t1);

        float t2 = +2.0 * (w * y - z * x);
        t2 = t2>1.0 ? 1.0 : t2;
        t2 = t2<-1.0 ? -1.0 : t2;
        ret.y = asin(t2);

        float t3 = +2.0 * (w * z + x * y);
        float t4 = +1.0 - 2.0 * (y * y + z * z);
        ret.z = atan2(t3, t4);
        return ret;
    }


    /**
     * @brief Find the min and the max from a array of float
     * would be nice to have this function independant to the type
     * 
     * @param array The array we want to search from
     * @param length The length of the array
     * @param min A pointer to the min value
     * @param max A pointer to the max value
     */
    static void minMaxFromArray (const float* array, const int length, int& min_id, int& max_id) {
        min_id = 0;
        max_id = 0;
        for (int i=1; i < length ; i++){
            if(array[i] < array[min_id])
                min_id = i;
            else if (array[i] > array[max_id])
                max_id = i;
        }
    }


};

#endif
