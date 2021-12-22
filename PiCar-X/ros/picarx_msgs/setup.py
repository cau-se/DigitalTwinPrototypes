from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    name='picarx_msgs',
    packages=['msg'],
    requires=['std_msgs', 'message_generation'])

setup(**setup_args)
