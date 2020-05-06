#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['interactive_world_creator'],
   package_dir={'interactive_world_creator': 'ros/src/interactive_world_creator'}
)

setup(**d)
