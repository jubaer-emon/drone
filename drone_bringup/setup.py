from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_bringup'

def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        install_path = os.path.join('share', package_name, path)
        file_list = [os.path.join(path, f) for f in filenames]
        if file_list:
            paths.append((install_path, file_list))
    return paths

data_files = [
    # required
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    # launch files
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')),

    # worlds
    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*.sdf')),
]

data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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
        ],
    },
)
