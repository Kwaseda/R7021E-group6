from setuptools import find_packages, setup

package_name = 'test_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dominiques-laptop',
    maintainer_email='domadd-2@student.ltu.se',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sender_node=test_pkg.sender:main',  # Write it just like the variable name used in the sender.py file 
            'receiver_node=test_pkg.receiver:main',
        ],
    },
)
