from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cobot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Make the package discoverable by ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Include launch files if any
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files if any
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include URDF or meshes if needed
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student14',
    maintainer_email='student14@todo.todo',
    description='6DOF collaborative robot simulation and control',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Python node entry point
            'cobot_control_node = cobot_pkg.cobot_control_node:main',
        ],
    },
)
