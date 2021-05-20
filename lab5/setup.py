from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
package_name = 'lab5'

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
        (os.path.join('share', package_name), glob('lab5/*')),
        (os.path.join('share', package_name), glob('*.json')),
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
            'service = lab5.jint_srv:main',
            'client = lab5.jint:main',
            'service_oint = lab5.oint_srv:main',
            'client_oint = lab5.oint:main',
        ],
    },
)
