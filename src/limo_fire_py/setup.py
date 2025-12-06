from setuptools import find_packages, setup

package_name = 'limo_fire_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/fire_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mingeun',
    maintainer_email='sonbaloo.naver.com@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'bt_visualizer = limo_fire_py.bt_visualizer:main',
		'bt_runner = limo_fire_py.bt_runner:main',
        'pygame_visualizer = limo_fire_py.pygame_visualizer:main',
		'fire_sensor_reader = limo_fire_py.fire_sensor_reader:main',
		'fire_safety_controller = limo_fire_py.fire_safety_controller:main',
		'fire_alarm_notifier = limo_fire_py.fire_alarm_notifier:main',
        'fire_sensor_publisher = limo_fire_py.fire_sensor_publisher:main',
        ],
    },
)
