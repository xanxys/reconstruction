#pragma once

#include <string>

#include <jsoncpp/json/json.h>

namespace recon {

// Call specified external Json-I/O program.
// WARNING: This function is not secure. Some prog_path string
// can cause arbitrary program execution.
Json::Value call_external(const std::string& prog_path, Json::Value arg);

}  // namespace
