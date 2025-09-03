import os
import sys
from setuptools import setup, find_packages
from setuptools.dist import Distribution

class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name
    """
    def has_ext_modules(self):
        return True

path_to_file = os.path.relpath(os.path.join("..", "pointmatcher", "PointMatcher.h"))


version = None

# (!) Limitation: This approach to fetching the POINTMATCHER_VERSION only work if the user execute
# python -m build --wheel
# It will break when using the default
# python -m build
try:
    with open(path_to_file, "r", encoding="utf-8") as f:
        for line in map(str.strip, f):
            if line.startswith("#define POINTMATCHER_VERSION"):
                version = line.split()[2].replace("\"", "")
                break
except FileNotFoundError:
    print(f"Cannot find: '{path_to_file}'. Probably reason is you run 'python -m build', but not 'python -m build --no-isolation --wheel'", file=sys.stderr)
    raise

if version is None:
    raise RuntimeError(f"Cannot find a version string in the file: '{path_to_file}'")


setup(packages=find_packages(include=["pypointmatcher*"]),
    package_data={"": ["*.so"]},
    version=version,
    distclass=BinaryDistribution
 )