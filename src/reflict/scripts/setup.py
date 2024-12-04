from setuptools import setup

package_name = 'reflict'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 Python package example',
    license='Apache License 2.0',
    entry_points={
    'console_scripts': [
        'scan_node = reflict.scan_node:main',
    ],
},

)
