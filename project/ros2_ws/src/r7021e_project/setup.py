from glob import glob

from setuptools import find_packages, setup

package_name = 'r7021e_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Addo',
    maintainer_email='domadd-2@student.ltu.se',
    description='R7021E project: MPC for a TurtleBot3 Burger carrying a plank, built on the Lab 2 controller.',
    license='MIT',

    entry_points={
        'console_scripts': [
            'mpc_node=r7021e_project.mpc_node:main',
        ],
    },
)
