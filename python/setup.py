from setuptools import setup, find_packages
from setuptools.dist import Distribution

class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name"""
    def has_ext_modules(self):
        return True

setup(packages=find_packages(include=["pypointmatcher*"]),
    install_requires=["numpy>=1.20"],
    package_data={"": ["*.so"]},
    distclass=BinaryDistribution
 )