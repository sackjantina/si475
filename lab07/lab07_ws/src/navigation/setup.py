import os
from setuptools import setup
from glob import glob
	
package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name) + '/map', glob('map/*')),
        (os.path.join('share', package_name) + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m233144',
    maintainer_email='m233144@usna.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation = navigation.navigation:main'
        ],
    },
)
