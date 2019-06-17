from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='esiaf_doa',
    version='0.0.1',
    description='A simple Direction of Arrival node for the esiaf_ros framework',
    url='---none---',
    author='rfeldhans',
    author_email='rfeldh@gmail.com',
    license='---none---',
    install_requires=[
        'pyroomacoustics'
    ],
    packages=['esiaf_doa']

)

setup(**setup_args)