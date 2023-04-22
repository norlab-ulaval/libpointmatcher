from setuptools import setup, find_packages
from setuptools.dist import Distribution

class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name
    """
    def has_ext_modules(self):
        return True

path_to_file = "../pointmatcher/PointMatcher.h"

version = None

with open(path_to_file, "r", encoding="utf-8") as f:
    for line in map(str.strip, f):
        if line.startswith("#define POINTMATCHER_VERSION"):
            version = line.split()[2].replace("\"", "")
            break

if version is None:
    raise RuntimeError(f"Cannot find a version string in the file: '{path_to_file}'")


setup(packages=find_packages(include=["pypointmatcher*"]),
    install_requires=["numpy>=1.20"],
    package_data={"": ["*.so"]},
    version=version,
    distclass=BinaryDistribution
 )