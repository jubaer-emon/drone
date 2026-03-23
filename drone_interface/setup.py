from setuptools import find_packages, setup

package_name = 'drone_interface'

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
    maintainer='emon',
    maintainer_email='emon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_odom_bridge = drone_interface.px4_odom_bridge:main',
            'wait_for_px4 = drone_interface.wait_for_px4:main',
        ],
    },
)
