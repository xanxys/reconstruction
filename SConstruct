# default header
import os
env = Environment(
	CXX = "clang++",
	CXXFLAGS = '-std=c++11 -Wno-deprecated -Wno-attributes',
	CCFLAGS = ['-O3', '-g'],
	CPPPATH = [
		'/usr/include/eigen3',
		'/usr/include/pcl-1.6',
		'/usr/include/vtk',
		'/usr/include/ni',
	])
env['ENV']['TERM'] = os.environ['TERM']

# project specific code
env.Program(
	'future_lens',
	source = [
		'main.cpp',
		],
	LIBS = [
		'libboost_system-mt',
		'libboost_thread-mt',
		'libdl',
		'libopencv_core',
		'libopencv_highgui',
		'libopencv_imgproc',
		'libpcl_common',
		'libpcl_features',
		'libpcl_visualization',
		'libpcl_filters',
		'libpcl_io',
		'libpcl_kdtree',
		'libpcl_search',
		'libpcl_segmentation',
		'libpthread',
		'libOpenNI',
		])
