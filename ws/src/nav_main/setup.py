from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='angelcervant@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holonomic_controller = nav_main.holonomic_controller:main',
        ],
    },
)
