#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "frame_belief.h"

enum class Direction {
	XP,
	XN,
	YP,
	YN,
	ZP,
	ZN
};


enum class VoxelState {
	OCCUPIED,
	EMPTY
};


class VoxelDescription {
public:
	VoxelDescription();
public:
	bool guess;
	VoxelState state;
	Eigen::Vector3f average_image_color;
};


class ManhattanBelief {
public:
	static std::vector<std::shared_ptr<ManhattanBelief>> expand(const FrameBelief& frame);
	
	ManhattanBelief(const ManhattanBelief& that);
	ManhattanBelief(const FrameBelief& frame, Eigen::Matrix3f camera_loc_to_world);

	Eigen::Vector2f projectToRGBCameraScreen(Eigen::Vector3f pos_world);

	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailed() const;
	float getVoxelSize() const;
private:
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailedWithoutGuess() const;
	static std::shared_ptr<ManhattanBelief> align(const FrameBelief& frame);

public:
	// Put this before all other members to initialize first,
	// since logging is used in SceneAnalyzer's initializer's list.
	mutable std::ostringstream log;

	FrameBelief frame;

	// aligned cloud
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;

	Eigen::Matrix3f camera_loc_to_world;
	Eigen::Matrix3f world_to_camera_loc;
	float voxel_size;
};
