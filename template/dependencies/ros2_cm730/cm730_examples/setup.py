from setuptools import setup

package_name = 'cm730_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'head_zero',
        'torque_off',
        'ping_motors'
    ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    maintainer='Sander van Dijk',
    maintainer_email='sgvandijk@gmail.com',
    keywords=['ROS'],
    description='Examples using the CM730-controller',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'head_zero = head_zero:main',
            'torque_off = torque_off:main',
            'ping_motors = ping_motors:main'
        ],
    },
)
