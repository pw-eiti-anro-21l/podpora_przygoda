import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'lab4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('urdf/*.rviz')),
        (os.path.join('share', package_name), glob('*.json'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anna',
    maintainer_email='ania@example.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nonkdl = lab4.nonkdl_dkin:main',
            'kdl = lab4.kdl_dkin:main',
            'service = lab4.jint_controle_srv:main',
            'client = lab4.jint:main',
        ],
    },
)
