#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "frame_belief.h"
#include "manhattan_belief.h"


// Finite plane (quad) with texture.
// It's an interface between known and unknown regions of the scene.
// (walls etc.), and NOT part of objects like box.
//
// Local coords: (z+ is normal)
// x: [-size/2,size/2]
// y: [-size/2,size/2]
// z: 0
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


class OrientedBox {
public:
	OrientedBox(
		Eigen::Vector3f position,
		float ry,
		Eigen::Vector3f size,
		Eigen::Vector3f color,
		bool valid);

	Eigen::Vector3f getPosition() const;
	Eigen::Vector3f getSize() const;
	Eigen::Vector3f getColor() const;
	float getRotationY() const;
	bool getValid() const;
private:
	float ry;

	Eigen::Vector3f position;
	Eigen::Vector3f size;
	Eigen::Vector3f color;
	bool valid;
};


class FloorBelief {
public:
	FloorBelief(const FloorBelief& that);
	FloorBelief(const ManhattanBelief& manhattan, int index);
	static std::vector<std::shared_ptr<FloorBelief>> expand(const ManhattanBelief& manhattan);
public:
	// Put this before all other members to initialize first,
	// since logging is used in SceneAnalyzer's initializer's list.
	mutable std::ostringstream log;

	ManhattanBelief manhattan;

	int index;
};

class WallBelief {
};

class ObjectsBelief {
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
	SceneBelief(FloorBelief& floor);

	// attribs
	std::string getLog();
	std::vector<TexturedPlane> getPlanes();
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getCloud();
	Eigen::Matrix3f getCameraLocalToWorld();
	float getFloor();

	cv::Mat getRGBImage();
	// return meter in f32
	cv::Mat getDepthImage();

	cv::Mat renderRGBImage();
	std::map<std::tuple<int, int, int>, VoxelState> getVoxels();
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailed();

	std::vector<OrientedBox> getObjects();
protected:
	// Extract textured plane from a plane with Manhattan normal.
	// index, distance means the coordinate of dir.
	// (e.g. XN,XP -> index, distance is x coord of the plane)
	TexturedPlane extractPlane(int index, float coord, Direction dir);

	// Synthesize complete texture from RGB image and unreliable mask.
	static cv::Mat synthesizeTexture(const cv::Mat image, const cv::Mat mask);
	static cv::Mat growTexture(const cv::Mat core, int width, int height);
protected:
	FrameBelief frame;
	ManhattanBelief manhattan;
	FloorBelief floor;
};
