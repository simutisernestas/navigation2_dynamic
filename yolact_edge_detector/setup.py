from setuptools import setup
import glob

package_name = 'yolact_edge_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/weights', glob.glob('weights/*'))
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
            'yolact_edge_node = yolact_edge_detector.yolact_edge_node:main'
        ],
    },
)