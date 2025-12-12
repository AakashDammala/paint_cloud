import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'paint_cloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        
        # Install urdf files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

        # Install mesh files
        (os.path.join('share', package_name, 'urdf/meshes/visual'), glob('urdf/meshes/visual/*')),
        (os.path.join('share', package_name, 'urdf/meshes/collision'), glob('urdf/meshes/collision/*')),

        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Install rviz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aakash',
    maintainer_email='dammala.aakash@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'paint_cloud = paint_cloud.paint_cloud:main',
        ],
    },
)
