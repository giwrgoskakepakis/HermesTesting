from setuptools import find_packages, setup

package_name = 'sensor_package'

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
    maintainer='giorgos',
    maintainer_email='giorgos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gyroscope_sensor = sensor_package.gyroscope_sensor:main', 
            'temperature_sensor = sensor_package.temperature_sensor:main',
            'voltage_sensor = sensor_package.voltage_sensor:main',
            'sole_pressure_sensor = sensor_package.sole_pressure_sensor:main'
        ],
    },
)
