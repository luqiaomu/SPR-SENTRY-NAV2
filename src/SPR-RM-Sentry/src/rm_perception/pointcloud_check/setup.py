from setuptools import find_packages, setup

package_name = 'pointcloud_check'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pcd_publisher_rviz.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dorothy',
    maintainer_email='dorothy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'pcd_publisher = pointcloud_check.pcd_publisher:main',
        ],
    },
)
