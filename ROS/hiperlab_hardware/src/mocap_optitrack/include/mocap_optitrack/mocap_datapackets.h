/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, University of Bonn, Computer Science Institute VI
 *  Author: Kathrin Gräve, 01/2011
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>

#ifndef __MOCAP_DATAPACKETS_H__
#define __MOCAP_DATAPACKETS_H__

#include <sys/types.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "hiperlab_rostools/mocap_output.h"
#include "Common/Math/Rotation.hpp"

using namespace std;

/// \brief Data object holding the position of a single mocap marker in 3d space
class Marker
{
  public:
    float positionX;
    float positionY;
    float positionZ;
};

class Pose
{
  public:
    struct {
      float x;
      float y;
      float z;
    } position;
    struct {
      float x;
      float y;
      float z;
      float w;
    } orientation;
};

/// \brief Data object holding information about a single rigid body within a mocap skeleton
class RigidBody
{
  public:
    RigidBody();
    ~RigidBody();

    int ID;
    
    Pose pose;

    int NumberOfMarkers;
    Marker *marker;

    const hiperlab_rostools::mocap_output get_ros_pose(int vehicleID);
    bool has_data();
};

/// \brief Data object describing a single tracked model
class ModelDescription
{
  public:
    ModelDescription();
    ~ModelDescription();

    string name;
    int numMarkers;
    string *markerNames;
};

class MarkerSet
{
  public:
    MarkerSet() : numMarkers(0), markers(0) {}
    ~MarkerSet() { delete[] markers; }
    char name[256];
    int numMarkers;
    Marker *markers;
};

/// \brief Data object holding poses of a tracked model's components
class ModelFrame
{
  public:
    ModelFrame();
    ~ModelFrame();

    MarkerSet *markerSets;
    Marker *otherMarkers;
    RigidBody *rigidBodies;

    int numMarkerSets;
    int numOtherMarkers;
    int numRigidBodies;

    float latency;
};

/// \breif Version class containing the version information and helpers for comparison.
class Version
{
  public:
    Version();
    Version(int major, int minor, int revision, int build);
    Version(const std::string& version);
    ~Version();

    void setVersion(int major, int minor, int revision, int build);
    const std::string& getVersionString();
    bool operator > (const Version& comparison);
    bool operator == (const Version& comparison);

    int v_major;
    int v_minor;
    int v_revision;
    int v_build;
    std::string v_string;
};

/// \brief Parser for a NatNet data frame packet
class MoCapDataFormat
{
  public:
    MoCapDataFormat(const char *packet, unsigned short length);
    ~MoCapDataFormat();

    /// \brief Parses a NatNet data frame packet as it is streamed by the Arena software according to the descriptions in the NatNet SDK v1.4
    void parse ();

    void setVersion(int nver[4], int sver[4])
    {
      NatNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
      ServerVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
    }

    const char *packet;
    unsigned short length;

    int frameNumber;
    ModelFrame model;

    Version NatNetVersion;
    Version ServerVersion;

  private:
    void seek(size_t count);
    template <typename T> void read_and_seek(T& target)
    {
        target = *((T*) packet);
        seek(sizeof(T));
    }
};

#endif  /*__MOCAP_DATAPACKETS_H__*/
