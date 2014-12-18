#pragma once

#include <recog/scene_asset_bundle.h>

namespace recon {

// Populate bundle with non-realistic fixed toy objects and rooms.
// Useful for integration testing of recon and manual UE4 testing
// (and demonstration).
void populateToyScene(SceneAssetBundle& bundle);

}  // namespace
