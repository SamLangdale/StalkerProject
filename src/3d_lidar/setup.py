from glob import glob
from setuptools import find_packages, setup

package_name = '3d_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/model', glob('../../model/*.urdf')),
        ('share/' + package_name + '/model/meshes', glob('../../model/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='samlangdale13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_3d = 3d_lidar.scan_3d:main',
            'joint_spinner = 3d_lidar.joint_spinner:main',
            'tf_broadcast = 3d_lidar.TF_broadcast:main',
        ],
    },
)
