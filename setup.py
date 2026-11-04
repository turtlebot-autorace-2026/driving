from setuptools import find_packages, setup


package_name = 'driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/lane_yolo26n_seg.pt', 'models/README.md']),
    ],
    install_requires=['setuptools'],
    python_requires='>=3.8',
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Driving node skeleton for ROS 2 Galactic.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'driving = driving.driving:main',
            'driving_debug = driving.driving_debug:main',
        ],
    },
)
