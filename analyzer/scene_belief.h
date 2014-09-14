#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "floor_belief.h"
#include "frame_belief.h"
#include "manhattan_belief.h"
#include "voxel_traversal.h"
#include "wall_belief.h"


// Finite plane (quad) with texture.
// It's an interface between known and unknown regions of the scene.
// (walls etc.), and NOT part of objects like box.
//
// Local coords: (z+ is normal)
// x: [-size/2,size/2]
// y: [-size/2,size/2]
// z: 0
//
// TODO: wanna hide this detail
// Texture Coordinates
// * x: x
// * y: inverted
class TexturedPlane {
public:
	TexturedPlane(float size, cv::Mat texture, float offset, Direction normal);

	Eigen::Matrix3f getLocalToWorld() const;

public:
	Direction normal;
	Eigen::Vector3f center;
	cv::Mat texture;
	const float size;
};


// A coherent set of belief about the scene, which may or may not be
// visible. It's a node of search tree.
//
// Belief is onion-like layerd structure.
// Outer belief is dependent on all inner beliefs.
//
// You might think that each belief should be independent (like terms in prolog),
// and solved by some kind of interpreter, but I didn't choose that design:
// * not sure we can "solve" such complex system in any kind of systematic way
// * need to care about belief dependencies all the time, which is pain
class SceneBelief {
public:
	SceneBelief(const WallBelief& wall);

	// attribs
	std::string getLog() const;
	std::vector<TexturedPlane> getPlanes() const;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getCloud() const;
	Eigen::Matrix3f getCameraLocalToWorld() const;
	float getFloor() const;

	cv::Mat getRGBImage() const;
	// return meter in f32
	cv::Mat getDepthImage() const;

	cv::Mat renderRGBImage() const;
	std::map<VoxelIndex, VoxelDescription> getVoxelsDetailed() const;

	std::vector<OrientedBox> getObjects() const;
protected:
	std::vector<OrientedBox> sampleObjects() const;
	
	// Extract textured plane from a plane with Manhattan normal.
	// index, distance means the coordinate of dir.
	// (e.g. XN,XP -> index, distance is x coord of the plane)
	TexturedPlane extractPlane(int index, float coord, Direction dir) const;

	// Synthesize complete texture from RGB image and unreliable mask.
	static cv::Mat synthesizeTexture(const cv::Mat image, const cv::Mat mask);
	static cv::Mat growTexture(const cv::Mat core, int width, int height);
protected:
	// Order is important. Members are initialized in this order.
	// So, to initialize references properly, we must copy floor first
	// and then create references into it. (NOT TO constructor arguments)
	WallBelief wall;
	FloorBelief& floor;
	ManhattanBelief& manhattan;
	FrameBelief& frame;
};
