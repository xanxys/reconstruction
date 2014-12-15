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
    return (
        os.path.isdir(path) and
        os.path.isfile(os.path.join(path, "meta.json")))


class InvalidSceneAsset(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


def copy_stripped_scene(src, dst):
    """
    Copy scene asset from src to dst, removing debug and checkpoint
    information if they exist.
    throws InvalidSceneAsset when encountering unexpected files etc.
    """
    def ignore(src, names):
        if os.path.abspath(src) == os.path.abspath(src):
            # root directory: ignore checkpoints
            return set(["checkpoints"])
        else:
            return set()
    logger.info("Copying stripped scene asset to %s", dst)
    copytree_wo_permission(src, dst, ignore)


def copytree_wo_permission(src, dst, ignore=None):
    """
    Recursively copy a directory tree, without trying to
    change permissions.
    """
    names = os.listdir(src)
    if ignore is not None:
        ignored_names = ignore(src, names)
    else:
        ignored_names = set()

    os.makedirs(dst)
    errors = []
    for name in names:
        if name in ignored_names:
            continue
        srcname = os.path.join(src, name)
        dstname = os.path.join(dst, name)
        try:
            if os.path.isdir(srcname):
                copytree_wo_permission(srcname, dstname, ignore)
            else:
                shutil.copyfile(srcname, dstname)
        # catch the Error from the recursive copytree so that we can
        # continue with other files
        except Error as err:
            errors.extend(err.args[0])
        except EnvironmentError as why:
            errors.append((srcname, dstname, str(why)))
    if errors:
        raise Error(errors)


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
        logger.info("Deleting existing directory %s", package_path)
        shutil.rmtree(package_path)
    logger.info("Creating experiment package at %s", package_path)
    os.mkdir(package_path)

    package_meta = {}

    copy_stripped_scene(args.scene_asset, os.path.join(package_path, "scene"))
    for quake in experiment["quakes"]:
        # TODO: generate BG sound
        data_to_sound.convert_acc_to_sound

    json.dump(package_meta, open(os.path.join(package_path, "meta.json"), "w"))
