from setuptools import setup
from glob import glob

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
        ('share' + package_name + '/map', glob('map/*')),
        ('share' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m235580',
    maintainer_email='m235580@usna.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localizer = localization.localizer:main'
        ],
    },
)
