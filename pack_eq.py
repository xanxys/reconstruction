#!/bin/python3
# Create an "experiment package" from "scene asset" and "experiment data".
# Package dir structure:
#  /
#  |-scene
#    |-meta.json
#    |-scene
#    | |-...
#    |-EqSound-0.wav
#    |- ...
#    |-collision-0.av
#    |- ...
import sys
sys.path.append('sound')
import argparse
import logging
import os
import os.path
import json
import shutil
import data_to_sound

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def is_scene_asset_shallow(path):
    """
    Perform a shallow check to see if the path
    is indeed scene asset generated by recon.
    """
    return (os.path.isdir(path) and
        os.path.isfile(os.path.join(path, "small_data.json")))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="""
Generate experiment package from the following:
1. Scene Asset (directory), generated by recon
2. experiment config (json file)

Result will be generated in another directory.
""",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--scene-asset', required=True, type=str,
        help='Directory path containing scene asset.')
    parser.add_argument(
        '--experiment', required=True, type=str,
        help='Path to experiment setup json file.')
    parser.add_argument(
        '--output', required=True, type=str,
        help='Output wave file path for soundscape simulation.')

    args = parser.parse_args()

    if not is_scene_asset_shallow(args.scene_asset):
        logger.warn("%s doesn't seem to be a scene asset" % args.scene_asset)
        sys.exit(1)

    experiment = json.load(open(args.experiment))

    # Create package
    package_path = args.output
    if os.path.isfile(package_path):
        logger.warn("package path %s already exists, and it's not a package; aborting", package_path)
        sys.exit(1)
    elif os.path.isdir(package_path):
        shutil.rmtree(package_path)

    package_meta = {}

    shutil.copytree(args.scene_asset, os.path.join(package_path, "scene"))
    for quake in experiment["quakes"]:
        # TODO: generate BG sound
        data_to_sound.convert_acc_to_sound

    json.dump(package_meta, open(os.path.join(package_path, "meta.json"), "w"))
