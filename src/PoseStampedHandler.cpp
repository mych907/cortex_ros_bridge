/*
	Copyright 2015 Tyler Sorey, ARL, University of Nevada, Reno, USA

    This file is part of cortex_bridge.

    cortex_bridge is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    cortex_bridge is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "cortex.h"
#include "geometry_msgs/PoseStamped.h"

#define piDiv180 0.01745329251

geometry_msgs::PoseStamped CreatePoseStamped ( sFrameOfData* FrameOfData, int frame_index, const char* body, float* origin_offset )
{
	// initialize variables
	double pos_x, pos_y, pos_z, roll, pitch, yaw;
	int numSegments;
	bool valid_data = true;
        std::stringstream convert;
	tSegmentData* Segments = FrameOfData->BodyData[frame_index].Segments;
	numSegments = FrameOfData->BodyData[frame_index].nSegments;

	geometry_msgs::PoseStamped MoCapGPS_msg;
	MoCapGPS_msg.header.stamp = ros::Time::now();
	MoCapGPS_msg.header.frame_id = "/world";

        std::__cxx11::basic_stringstream<char>::__string_type subject_name = {0,};
	if ( Segments != NULL )
	{
		for ( int i = 0; i < numSegments; ++i )
		{
            // subject name
            convert << FrameOfData->BodyData[i].szName;
            subject_name = convert.str();
            convert.str( std::string() );
            convert.clear();

            // Run only if the subject name matches the body of interest
            if (subject_name == "AGV")
            {
                // get body data
                pos_x = (double)Segments[i][0]; // x
                pos_y = (double)Segments[i][1]; // y
                pos_z = (double)Segments[i][2]; // z

                roll = (piDiv180)*Segments[i][3]; // roll (in rad)
                pitch = (piDiv180)*Segments[i][4]; // pitch (in rad)
                yaw = (piDiv180)*Segments[i][5]; // yaw (in rad)

                pos_x -= origin_offset[0];
                pos_y -= origin_offset[1];
                pos_z -= origin_offset[2];
                roll = ( roll - origin_offset[3] );
                pitch = ( pitch - origin_offset[4] );
                yaw = ( yaw - origin_offset[5] );

            }
            tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

            MoCapGPS_msg.pose.position.x = pos_x;
            MoCapGPS_msg.pose.position.y = pos_y;
            MoCapGPS_msg.pose.position.z = pos_z;
            MoCapGPS_msg.pose.orientation.x = quat[0];
            MoCapGPS_msg.pose.orientation.y = quat[1];
            MoCapGPS_msg.pose.orientation.z = quat[2];
            MoCapGPS_msg.pose.orientation.w = quat[3];
        }
        return MoCapGPS_msg;
    }
}
