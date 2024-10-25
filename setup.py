from setuptools import setup

package_name = 'yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Points to the yolo_pkg directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_camera_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Romero',
    maintainer_email='sergioromero48@outlook.com',
    description='Simple ROS2 package for YOLO object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_test = yolo_pkg.yolo_test:main',  # Reference your yolo_test.py file
        ],
    },
)
