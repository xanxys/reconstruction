#include <iostream>
#include <tuple>
#include <vector>

#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/range/irange.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <sensor_msgs/PointField.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr convertPyToPCL(boost::python::object points) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	for(int i : boost::irange(0, (int)boost::python::len(points))) {
		boost::python::object point = points[i];

		pcl::PointXYZ p;
		p.x = boost::python::extract<double>(point["x"]);
		p.y = boost::python::extract<double>(point["y"]);
		p.z = boost::python::extract<double>(point["z"]);
		cloud->points.push_back(p);
	}
	return cloud;
}

boost::python::dict extractField(const std::vector<sensor_msgs::PointField> schema, const uint8_t* ptr) {
	boost::python::dict vertex;
	for(const auto& field : schema) {
		assert(field.count == 1);
		if(field.datatype == sensor_msgs::PointField::FLOAT32) {
			vertex[field.name] = *reinterpret_cast<const float*>(ptr + field.offset);
		}
	}
	return vertex;
}

boost::python::dict convertMeshToPy(const pcl::PolygonMesh& triangles) {
	std::cout << "Output Mesh Fields" << std::endl;
	for(const auto& field : triangles.cloud.fields) {
		std::cout << field.name << "@" << field.offset <<
			":" << std::to_string(field.datatype) << "*" << field.count << std::endl;
	}
	std::cout <<
		"w=" << triangles.cloud.width << " / h=" << triangles.cloud.height <<
		"row_step=" << triangles.cloud.row_step << " / pt_step=" << triangles.cloud.point_step << std::endl;
	boost::python::list vertices;
	assert(triangles.cloud.height == 1);
	for(const int i : boost::irange(0, (int)triangles.cloud.width)) {
		vertices.append(
			extractField(triangles.cloud.fields,
				triangles.cloud.data.data() + i * triangles.cloud.point_step));
	}

	boost::python::list faces;
	for(const auto& face : triangles.polygons) {
		boost::python::list face_py;
		for(const int index : face.vertices) {
			face_py.append(index);
		}
		faces.append(face_py);
	}

	boost::python::dict mesh;
	mesh["vertices"] = vertices;
	mesh["faces"] = faces;
	return mesh;
}

boost::python::object construct_mesh(boost::python::object points) {
	auto cloud = convertPyToPCL(points);

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.05);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	return convertMeshToPy(triangles);
}

BOOST_PYTHON_MODULE(mist) {
	boost::python::def("construct_mesh", &construct_mesh);
}
