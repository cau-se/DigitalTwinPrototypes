from setuptools import setup

setup(
    name='simple_emulator',
    version='1.0',
    description='A simple Emulator to demonstrate integration tests of RS232 with Docker',
    license="Apache 2.0",
    author='Alexander Barbie',
    author_email='alexanderbarbie@gmx.de',
    url="https://github.com/",
    package_dir={'': 'src'},
    packages=['simple_emulator'],  # same as name
    install_requires=['pyserial'],  # external packages as dependencies
)
