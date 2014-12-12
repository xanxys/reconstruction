#include "util.h"

#include <vector>

#include <boost/range/irange.hpp>
#include <gtest/gtest.h>

TEST(getCCTest, EmptyGraph) {
	std::set<int> verts;
	std::map<int, std::set<int>> adj;
	EXPECT_TRUE(recon::getCC(verts, adj).empty());
}

TEST(getCCTest, TwoCC) {
	std::set<int> verts{1, 2};
	std::map<int, std::set<int>> adj{
		{1, {}},
		{2, {}}
	};
	const auto ccs = recon::getCC(verts, adj);
	EXPECT_EQ(2, ccs.size());
	EXPECT_EQ(1, ccs[0].size());
	EXPECT_EQ(1, ccs[1].size());
}

TEST(getCCTest, OneCC) {
	std::set<int> verts{1, 2};
	std::map<int, std::set<int>> adj{
		{1, {2}},
		{2, {1}}
	};
	const auto ccs = recon::getCC(verts, adj);
	EXPECT_EQ(1, ccs.size());
	EXPECT_EQ(2, ccs[0].size());
}
