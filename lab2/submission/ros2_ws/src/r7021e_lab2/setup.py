from setuptools import find_packages, setup

package_name = 'r7021e_lab2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab2_launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/lab2.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Addo',
    maintainer_email='domadd-2@student.ltu.se',
    description='Lab 2: nonlinear MPC for the TurtleBot3 Burger, built with do-mpc.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpc_node=r7021e_lab2.mpc_controller:main',
        ],
    },
)
