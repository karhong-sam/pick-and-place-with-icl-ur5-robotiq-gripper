## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['icl_phri_ur5_control'],
    package_dir={'': 'src'},
    requires=['actionlib', 'rospy', 'moveit_commander', 'icl_phri_robotiq_control']
)

setup(**setup_args)