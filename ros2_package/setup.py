from setuptools import setup

package_name = 'ar_robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AR Robot',
    maintainer_email='user@example.com',
    description='AR Robot motor control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = motor_controller:main',
            'test_motor_control = test_motor_control:main',
            'mission_controller = mission_controller:main',
        ],
    },
)
