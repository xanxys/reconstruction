#include "scene_asset_bundle.h"

#include <fstream>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

namespace fs = boost::filesystem;

TEST(SceneAssetBundle, CleanDirectoryWorksForEmptyDirectory) {
	// Make sure there's no directory.
	const fs::path target = "/tmp/recon-test-bundle-empty";
	fs::remove_all(target);

	recon::SceneAssetBundle::cleanDirectory(target);
	EXPECT_TRUE(fs::exists(target));
	EXPECT_TRUE(fs::is_directory(target));
	EXPECT_TRUE(fs::exists(target / fs::path("checkpoints")));
	EXPECT_TRUE(fs::is_directory(target / fs::path("checkpoints")));
}

TEST(SceneAssetBundle, CleanDirectoryCreatesCheckpointsDirectory) {
	// Setup directory that contains files & directories
	const fs::path target = "/tmp/recon-test-bundle-cp";
	fs::remove_all(target);
	fs::create_directory(target);

	recon::SceneAssetBundle::cleanDirectory(target);
	EXPECT_TRUE(fs::exists(target));
	EXPECT_TRUE(fs::is_directory(target));
	// "checkpoints" dir must be created.
	EXPECT_TRUE(fs::exists(target / fs::path("checkpoints")));
	EXPECT_TRUE(fs::is_directory(target / fs::path("checkpoints")));
}

TEST(SceneAssetBundle, CleanDirectoryWorksForMessyDirectory) {
	// Setup directory that contains files & directories
	// /
	// |- hoge.txt
	// |- hoge_dir/
	// |- checkpoints
	//     |- foo.txt
	// hoge.txt, hoge_dir: must be removed
	// foo.txt: must be preserved
	const fs::path target = "/tmp/recon-test-bundle-messy";
	{
		fs::remove_all(target);
		fs::create_directory(target);
		std::ofstream hoge_txt((target / fs::path("hoge.txt")).string());
		fs::create_directory(target / fs::path("hoge_dir"));
		fs::create_directory(target / fs::path("checkpoints"));
		std::ofstream foo_txt((target / fs::path("checkpoints") / fs::path("foo.txt")).string());
	}

	recon::SceneAssetBundle::cleanDirectory(target);
	EXPECT_TRUE(fs::exists(target));
	EXPECT_TRUE(fs::is_directory(target));
	EXPECT_FALSE(fs::exists(target / fs::path("hoge.txt")));
	EXPECT_FALSE(fs::exists(target / fs::path("hoge_dir")));
	EXPECT_TRUE(fs::exists(target / fs::path("checkpoints")));
	EXPECT_TRUE(fs::is_directory(target / fs::path("checkpoints")));
	EXPECT_TRUE(fs::exists(target / fs::path("checkpoints/foo.txt")));
	EXPECT_FALSE(fs::is_directory(target / fs::path("checkpoints/foo.txt")));
}
