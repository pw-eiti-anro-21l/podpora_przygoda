import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'urdf_tutorial'

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
    (os.path.join('share', package_name), glob('urdf/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anna',
    maintainer_email='ania@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'state_publisher = urdf_tutorial.state_publisher:main'
        ],
    },
)
