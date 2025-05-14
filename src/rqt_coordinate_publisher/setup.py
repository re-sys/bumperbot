from setuptools import find_packages, setup

package_name = 'rqt_coordinate_publisher'

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
    maintainer='wufy',
    maintainer_email='wufy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'CoordinatePublisher = rqt_coordinate_publisher.plugin:CoordinatePublisher',
            'nav_ui = rqt_coordinate_publisher.nav_ui:main',
            'simulated_robot = rqt_coordinate_publisher.simulated_robot:main'
        ],
    },
)
