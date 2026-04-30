from setuptools import find_packages, setup

package_name = 'cylinder_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/cylinder_detector_pkg/msg', ['msg/CylinderTarget.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student07',
    maintainer_email='student07@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
    'console_scripts': [
        'yolo_realsense_node = cylinder_detector_pkg.yolo_realsense_node:main',
    ],
},
)
