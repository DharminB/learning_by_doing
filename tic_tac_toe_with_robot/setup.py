#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['tic_tac_toe_with_robot'],
   package_dir={'tic_tac_toe_with_robot': 'common/tic_tac_toe_with_robot'}
)

setup(**d)
