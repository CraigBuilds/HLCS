from setuptools import find_packages, setup

package_name = 'hlcs'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/qml', ['hlcs/qml/main.qml']),
    ],
    install_requires=['setuptools', 'asyncua', 'PySide6'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='High Level Control System - ROS2 package with OPC UA integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = hlcs.sim:main',
            'driver = hlcs.driver:main',
            'gui = hlcs.gui:main',
        ],
    },
)
