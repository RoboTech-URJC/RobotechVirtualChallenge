from setuptools import find_packages
from setuptools import setup

setup(
    name='openni2_camera',
    version='2.3.0',
    packages=find_packages(
        include=('openni2_camera', 'openni2_camera.*')),
)
