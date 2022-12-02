## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ck_utilities_py_node'],  #packages=['ck_utilities_py_node', 'ck_utilities_py_node.subnode'],
    package_dir={'': 'src'})

setup(**setup_args)