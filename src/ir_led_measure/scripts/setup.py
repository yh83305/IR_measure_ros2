from setuptools import setup

package_name = 'ir_led_measure'

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
            'camera_publisher = ir_led_measure.camera_publisher:main',
            'fusion = ir_led_measure.fusion:main',
            'camera_rosbag = ir_led_measure.camera_rosbag:main'
        ],
    },
)
