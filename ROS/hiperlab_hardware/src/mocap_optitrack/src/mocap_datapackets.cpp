#include "mocap_optitrack/mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <ros/console.h>
#include <tf/transform_datatypes.h>

using namespace std;

RigidBody::RigidBody() 
  : NumberOfMarkers(0), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

const hiperlab_rostools::mocap_output RigidBody::get_ros_pose(int vehicleID)
{
  hiperlab_rostools::mocap_output mocapOutMsg;
  mocapOutMsg.header.stamp = ros::Time::now();
  mocapOutMsg.vehicleID = vehicleID;
  // y & z axes are swapped in the Optitrack coordinate system
  mocapOutMsg.posx = pose.position.x;
  mocapOutMsg.posy = -pose.position.z;
  mocapOutMsg.posz = pose.position.y;
  // We use (scalalr, vector) as quaternion, ROS uses (vector, scalar)
  // also y & z axes are swapped in the Optitrack coordinate system
  mocapOutMsg.attq0 = pose.orientation.w;  // attq0 = w
  mocapOutMsg.attq1 = pose.orientation.x;  // attq1 = x
  mocapOutMsg.attq2 = -pose.orientation.z; // attq2 = y
  mocapOutMsg.attq3 = pose.orientation.y; // attq3 = z

  double r, p, y;
  Rotationd(mocapOutMsg.attq0, mocapOutMsg.attq1, mocapOutMsg.attq2, mocapOutMsg.attq3).ToEulerYPR(y,p,r);

  mocapOutMsg.attyaw = y;
  mocapOutMsg.attpitch = p;
  mocapOutMsg.attroll = r;

  return mocapOutMsg;
}

bool RigidBody::has_data()
{
    static const char zero[sizeof(pose)] = { 0 };
    return memcmp(zero, (char*) &pose, sizeof(pose));

}

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markerSets(0), otherMarkers(0), rigidBodies(0), 
    numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0),
    latency(0.0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markerSets;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

Version::Version()
  : v_major(0), v_minor(0), v_revision(0), v_build(0)
{
}

Version::Version(int major, int minor, int revision, int build)
  : v_major(major), v_minor(minor), v_revision(revision), v_build(build)
{
  std::ostringstream ostr;
  ostr << v_major << "." << v_minor << "." << v_revision << "." << v_build;
  v_string  = ostr.str();
}

Version::Version(const std::string& version)
  : v_string(version)
{
  std::sscanf(version.c_str(), "%d.%d.%d.%d", &v_major, &v_minor, &v_revision, &v_build);
}

Version::~Version()
{
}
void Version::setVersion(int major, int minor, int revision, int build)
{
  v_major = major;
  v_minor = minor;
  v_revision = revision;
  v_build = build;

}

const std::string& Version::getVersionString()
{
  return this->v_string;
}

bool Version::operator > (const Version& comparison)
{
  if (v_major > comparison.v_major)
    return true;
  if (v_minor > comparison.v_minor)
    return true;
  if (v_revision > comparison.v_revision)
    return true;
  if (v_build > comparison.v_build)
    return true;
  return false;
}
bool Version::operator == (const Version& comparison)
{
  return v_major == comparison.v_major
      && v_minor == comparison.v_minor
      && v_revision == comparison.v_revision
      && v_build == comparison.v_build;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length) 
  : packet(packet), length(length), frameNumber(0)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse()
{
  seek(4); // skip 4-bytes. Header and size.


  // parse frame number
  read_and_seek(frameNumber);
  ROS_DEBUG("Frame number: %d", frameNumber);

  // count number of packetsets
  read_and_seek(model.numMarkerSets);
  model.markerSets = new MarkerSet[model.numMarkerSets];
  ROS_DEBUG("Number of marker sets: %d\n", model.numMarkerSets);

  for (int i = 0; i < model.numMarkerSets; i++)
  {
    strcpy(model.markerSets[i].name, packet);
    seek(strlen(model.markerSets[i].name) + 1);

    ROS_DEBUG("Parsing marker set named: %s\n", model.markerSets[i].name);

    // read number of markers that belong to the model
    read_and_seek(model.markerSets[i].numMarkers);
    ROS_DEBUG("Number of markers in set: %d\n", model.markerSets[i].numMarkers);
    model.markerSets[i].markers = new Marker[model.markerSets[i].numMarkers];

    for (int k = 0; k < model.markerSets[i].numMarkers; k++)
    {
      // read marker positions
      read_and_seek(model.markerSets[i].markers[k]);
      float x = model.markerSets[i].markers[k].positionX;
      float y = model.markerSets[i].markers[k].positionY;
      float z = model.markerSets[i].markers[k].positionZ;
      ROS_DEBUG("\t marker %d: [x=%3.2f,y=%3.2f,z=%3.2f]", k, x, y, z);
    }
  }

  // read number of 'other' markers (cf. NatNet specs)
  read_and_seek(model.numOtherMarkers);
  model.otherMarkers = new Marker[model.numOtherMarkers];
  ROS_DEBUG("Number of markers not in sets: %d\n", model.numOtherMarkers);

  for (int l = 0; l < model.numOtherMarkers; l++)
  {
    // read positions of 'other' markers
    read_and_seek(model.otherMarkers[l]);
    float x = model.otherMarkers[l].positionX;
    float y = model.otherMarkers[l].positionY;
    float z = model.otherMarkers[l].positionZ;
    ROS_DEBUG("\t marker %d: [x=%3.2f,y=%3.2f,z=%3.2f]", l, x, y, z);
//    printf("marker %d: [x=%3.2f,y=%3.2f,z=%3.2f] \n", l, x, y, z);
  }

  // read number of rigid bodies of the model
  read_and_seek(model.numRigidBodies);
  ROS_DEBUG("Number of rigid bodies: %d\n", model.numRigidBodies);
  model.rigidBodies = new RigidBody[model.numRigidBodies];

  for (int m = 0; m < model.numRigidBodies; m++)
  {
    // read id, position and orientation of each rigid body
    read_and_seek(model.rigidBodies[m].ID);
    read_and_seek(model.rigidBodies[m].pose);

    // get number of markers per rigid body
    read_and_seek(model.rigidBodies[m].NumberOfMarkers);

    ROS_DEBUG("Rigid body ID: %d\n", model.rigidBodies[m].ID);
    ROS_DEBUG("Number of rigid body markers: %d\n", model.rigidBodies[m].NumberOfMarkers);

    if (model.rigidBodies[m].NumberOfMarkers > 0)
    {
      model.rigidBodies[m].marker = new Marker [model.rigidBodies[m].NumberOfMarkers];

      size_t byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(Marker);
      memcpy(model.rigidBodies[m].marker, packet, byte_count);
      seek(byte_count);

      // skip marker IDs
      byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(int);
      seek(byte_count);

      // skip marker sizes
      byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(float);
      seek(byte_count);
    }

    // skip mean marker error
    seek(sizeof(float) + 2);  //seek(sizeof(float));

    // 2.6 or later.
    if (NatNetVersion > Version("2.6"))
    {
      seek(sizeof(short));
    }

  }

  // TODO: read skeletons
  int numSkeletons = 0;
  read_and_seek(numSkeletons);

  // get latency
  read_and_seek(model.latency);
}
