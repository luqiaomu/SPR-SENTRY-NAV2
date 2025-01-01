from setuptools import find_packages, setup

package_name = 'my_open3d_ros2_package'

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
    maintainer='dyf',
    maintainer_email='huchunxu@guyuehome.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'open3d_ros2_node      = my_open3d_ros2_package.open3d_ros2_node:main',
        'remove      = my_open3d_ros2_package.remove:main',
        'cmd_vel_x_muliplier     = remove_multiplier.cmd_vel_x_muliplier:main',
        'remove_multiplier       = my_open3d_ros2_package.remove_multiplier:main', 
        ],
    },
)
