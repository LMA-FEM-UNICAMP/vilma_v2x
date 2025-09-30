from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vilma_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'website'), glob(os.path.join('website', '*.html'))),
        (os.path.join('share', package_name, 'website'), glob(os.path.join('website', '*.js'))),
         (os.path.join('share', package_name, 'website'), glob(os.path.join('website', '*.png')) + glob(os.path.join('images', '*.jpg'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Toffanetto',
    maintainer_email='gabriel.rocha@ieee.org',
    description='VILMA HMI',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
