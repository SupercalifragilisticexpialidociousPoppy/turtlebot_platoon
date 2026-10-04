from setuptools import find_packages, setup

package_name = 'platoon_controller'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'scan_filter = platoon_controller.scan_filter:main',
		'wall_driver = platoon_controller.wall_driver:main',
		'distance_controller = platoon_controller.distance_controller:main',
        ],
    },
)
