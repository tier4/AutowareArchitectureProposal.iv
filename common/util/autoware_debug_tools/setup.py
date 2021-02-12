from glob import glob
from setuptools import find_packages
from setuptools import setup
import os
package_name = 'autoware_debug_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('scripts/*.py'))
    ],
    install_requires=[ #todo
        'launch',
        'launch_ros',
        'launch_xml',
        'numpy',
        'setuptools',
        'rtree',
    ], 
    zip_safe=True,
    author='Kenji Miyake',
    author_email='kenji.miyake@tier4.jp',
    maintainer='Kenji Miyake',
    maintainer_email='kenji.miyake@tier4.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The autoware_debug_tools package.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose2tf = src.pose2tf:main',
            'stop_reason2pose = src.stop_reason2pose:main',
            'tf2pose = src.tf2pose:main'
        ],
    },
)
