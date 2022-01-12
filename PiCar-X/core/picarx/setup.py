from setuptools import setup, find_packages

setup(name = 'picarx',
  #packages=['picarx','picarx.interfaces', 'picarx.drivers', 'picarx.emulators'],
  packages=find_packages('picarx'),
  package_dir={'': 'src'},
  install_requires=['watchdog', 'smbus2', 'pyyaml', 'twisted']
)
