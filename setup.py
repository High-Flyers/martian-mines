from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='martian-mines',
    version='0.0.1',
    packages=find_packages('src'),
    package_dir={'':'src'},
    test_suite='tests',
    install_requires=requirements,
    extras_resuire={
        'dev': [
            'pytest'
        ]
    },
    description='Martian Mines category system for 2024 Droniada Challenge',
    author='High Flyers',
    author_email='highflyers.polsl@gmail.com',
    license='MIT'
)