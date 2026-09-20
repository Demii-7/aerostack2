from setuptools import setup

package_name = 'as2_experiment_aerpaw_multiuav'

setup(
    name=package_name,
    version='1.1.3',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_multirotor.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_gazebo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/aerpaw_digital_twin.launch.py']),
        ('share/' + package_name + '/missions', ['missions/triangle_formation.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='CVAR-UPM',
    author_email='cvar.upm3@gmail.com',
    description='Multi-UAV triangle formation experiment for AERPAW platform validation',
    license='BSD-3-Clause',
)