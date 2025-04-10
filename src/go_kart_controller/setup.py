from setuptools import setup, find_packages

package_name = 'go_kart_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
   data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/go_kart.launch.py']),
],

    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS2 package for the autonomous go-kart display and command nodes.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_node = go_kart_controller.nodes.display_node:main',
            'udp_forwarder_node = go_kart_controller.nodes.udp_forwarder_node:main'
        ],
    },
)
