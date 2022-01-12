from setuptools import setup, find_packages

setup(name = 'picarx_dcmotor_driver',
  #packages=['picarx','picarx.interfaces', 'picarx.drivers', 'picarx.emulators'],
  packages=find_packages('picarx_dcmotor_driver'),
  package_dir={'': 'src'},
  install_requires=[]
)
