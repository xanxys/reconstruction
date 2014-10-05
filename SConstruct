# default header
import os

if os.path.exists('/usr/bin/clang++'):
	compiler = 'clang++'
else:
	compiler = 'g++'

env = Environment(
	CXX = compiler,
	CXXFLAGS = '-std=c++11 -Wno-deprecated -Wno-attributes',
	CCFLAGS = ['-O3', '-g'],
	CPPPATH = [
	'/usr/include/eigen3',
	'/usr/include/GL',
	'/usr/include/GLFW',  # for EC2
	'/usr/include/ni',
	'/usr/include/pcl-1.7',
	'/usr/include/jsoncpp',
	'./',
	],
	LIBPATH = [
	],
	CPPDEFINES = [
		# 'ENABLE_USB_IO'
	]
	)
env['ENV']['TERM'] = os.environ.get('TERM', '')

# project specific code
LIBS = [
	'libboost_filesystem',
	'libboost_program_options',
	'libboost_system',
	'libboost_thread',
	'libCGAL',
	'libCGAL_Core',
	'libdl',
	'libGL',
	'libGLEW',
	'libglfw',
	'libgmp',
	'libjsoncpp',
	'libmpfr',
	'libopencv_core',
	'libopencv_features2d',
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
	'libprotobuf',
	'libpthread',
]

def exclude(f):
	return str(f).startswith('mist/') or str(f).startswith('SLIC-Superpixels')

# TODO: hack. doesn't work when proto new files are added
env.Command(
	['asset.pb.cc', 'asset.pb.h'],
	['proto/asset.proto'],
	'protoc --proto_path=proto --cpp_out ./ proto/asset.proto')

env.Depends(env.Glob('*.cpp'), ['asset.pb.cc', 'asset.pb.h'])
env.Depends(env.Glob('*/*.cpp'), ['asset.pb.cc', 'asset.pb.h'])

env.Program(
	'recon',
	source =
		[env.Object(f) for f in env.Glob('*.cpp') + env.Glob('*/*.cpp') + env.Glob("*/*.c") + env.Glob('*.cc')
			if not f.name.endswith('_test.cpp') and not exclude(f)],
	LIBS = LIBS)

program_test = env.Program(
	'recon_test',
	source =
		[env.Object(f) for f in env.Glob('*.cpp') + env.Glob('*/*.cpp') + env.Glob("*/*.c") + env.Glob('*.cc')
			if f.name != 'main.cpp' and not exclude(f)],
	LIBS = LIBS + ['libgtest', 'libgtest_main'])

env.Command('test', None, './' + program_test[0].path)
env.Depends('test', 'recon_test')
