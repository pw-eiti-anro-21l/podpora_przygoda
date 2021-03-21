from setuptools import setup
import os
from glob import glob

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paulina',
    maintainer_email='paulina.podpora.stud@pw.edu.pl',
    description='Laboratorium 1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab1_publisher = lab1.lab1.lab1_publisher:main'
        ],
    },
)
