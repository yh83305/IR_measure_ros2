import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='reflict',
    version='0.0.0',
    packages=find_packages(
        include=('reflict', 'reflict.*')),
)
