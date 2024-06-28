from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['send_gps'],
    package_dir={'': 'src'},
    install_requires=['scipy','pyproj']
)

setup(**setup_args)
