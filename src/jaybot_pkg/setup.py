from setuptools import find_packages, setup

package_name = 'jaybot_pkg'

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
    maintainer='jaegon',
    maintainer_email='jaegon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = jaybot_pkg.sonar.sonar_node:main',
            'camera_node = jaybot_pkg.camera.camera_node:main',
            'hardware_interface_node = jaybot_pkg.hardware.hardware_interface_node:main',
            'ackermann_node = jaybot_pkg.steering.ackermann_node:main',
            'encoder_node = jaybot_pkg.encoders.encoder_node:main',
        ],
    },
)
