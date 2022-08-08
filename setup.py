from setuptools import setup

package_name = 'simple_actions'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'simple_actions'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Library for easier use of rclpy actions',
    license='BSD',
)
