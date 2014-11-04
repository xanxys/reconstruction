#include "program_proxy.h"

#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>

TEST(CallExternalTest, EchoObject) {
	Json::Value arg;
	arg["foo"] = 123;
	arg["hoge"] = 45;

	Json::Value result = call_external("./program_proxy_test_echo.py", arg);
	EXPECT_EQ(arg, result);
}
