from setuptools import find_packages, setup

package_name = 'fruit_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # Replace with your actual name
    maintainer_email='your.email@example.com',  # Replace with your actual email
    description='ROS2 package for fruit detection using RGBD camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fruit_detector = fruit_detection.fruit_detector:main',
        ],
    },
)