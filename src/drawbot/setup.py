from glob import glob

from setuptools import find_packages, setup

package_name = 'drawbot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocki',
    maintainer_email='rockisking7@gmail.com',
    description='Two-link pen plotter arm (2x NEMA 17 + pen-lift servo) simulated in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = drawbot.controller_node:main',
            'trail = drawbot.trail_node:main',
            'draw = drawbot.draw_cli:main',
        ],
    },
)
