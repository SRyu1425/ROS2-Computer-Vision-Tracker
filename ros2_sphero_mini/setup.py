from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_sphero_mini'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=find_packages(exclude=['test']),
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Siwon Ryu',
    author_email='siwonryu1425@gmail.com',
    maintainer='Siwon Ryu',
    maintainer_email='siwonryu1425@gmail.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'sphero_node = ros2_sphero_mini.sphero_node:main',
            'tracker = ros2_sphero_mini.tracker_node:main',
            'gui = ros2_sphero_mini.controller_gui:main',
        ],
    },
)
