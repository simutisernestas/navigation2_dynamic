from setuptools import setup
import os, glob

package_name = 'yolect_edge_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('config/*.yaml')),
        ('share/' + package_name, glob.glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shengjian Chen',
    maintainer_email='csj15thu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolact_edge_node = yolect_edge_detector.yolact_edge_node:main'
        ],
    },
)
