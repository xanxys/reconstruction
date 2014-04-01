# default header
import os
env = Environment(
	CXX = "clang++",
	CXXFLAGS = '-std=c++11 -Wno-deprecated -Wno-attributes',
	CCFLAGS = ['-O3', '-g'],
	CPPPATH = [
	'/usr/include/eigen3',
	'/usr/include/GL',
	'/usr/include/ni',
	'/usr/include/pcl-1.6',
	'/usr/include/jsoncpp',
	],
	LIBPATH = [
	])
env['ENV']['TERM'] = os.environ['TERM']

# project specific code
LIBS = [
	'libboost_system-mt',
	'libboost_thread-mt',
	'libdl',
	'libGL',
	'libGLEW',
	'libglfw',
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
	'libpthread',
]

env.Program(
	'recon',
	source =
		[env.Object(f) for f in env.Glob('*.cpp') + env.Glob('*/*.cpp') + env.Glob("*.c")
			if not f.name.endswith('_test.cpp')],
	LIBS = LIBS)

program_test = env.Program(
	'recon_test',
	source =
		[env.Object(f) for f in env.Glob('*.cpp') + env.Glob('*/*.cpp') + env.Glob("*.c")
			if f.name != 'main.cpp'],
	LIBS = LIBS + ['libgtest', 'libgtest_main'])

env.Command('test', None, './' + program_test[0].path)
env.Depends('test', 'recon_test')
