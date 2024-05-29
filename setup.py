from distutils.core import setup
from setuptools import find_packages
from catkin_pkg.python_setup import generate_distutils_setup

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup_args = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'},
    test_suite='tests',
    install_requires=requirements,
    extras_resuire={
        'dev': [
            'pytest'
        ]
    },
)

setup(**setup_args)
