# default header
import os
env = Environment(
	CXX = "clang++",
	CXXFLAGS = '-std=c++11 -Wno-deprecated -Wno-attributes',
	CCFLAGS = ['-O3', '-g'],
	CPPPATH = [
		'/usr/include/eigen3',
		'/usr/include/pcl-1.6',
		'/usr/include/ni',
	],
	LIBPATH = [
	])
env['ENV']['TERM'] = os.environ['TERM']

# project specific code
env.Program(
	'recon',
	source = Glob('*.cpp') + Glob('*.c'),
	LIBS = [
		'libboost_system-mt',
		'libboost_thread-mt',
		'libdl',
		'libjsoncpp',
		'libopencv_core',
		'libopencv_highgui',
		'libopencv_imgproc',
		'libOpenNI',
		'libpcl_common',
		'libpcl_features',
		'libpcl_filters',
		'libpcl_io',
		'libpcl_kdtree',
		'libpcl_search',
		'libpcl_segmentation',
		'libpcl_visualization',
		'libpthread',
		])
