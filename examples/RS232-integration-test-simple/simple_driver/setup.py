from setuptools import setup

setup(
    name='simple_driver',
    version='1.0',
    description='A simple driver to demonstrate integration tests of RS232 interfaces with Docker',
    license="MIT",
    author='Alexander Barbie',
    author_email='alexanderbarbie@gmx.de',
    url="https://github.com/",
    package_dir={'': 'src'},
    packages=['simple_driver'],  # same as name
    install_requires=['pyserial'],  # external packages as dependencies
)