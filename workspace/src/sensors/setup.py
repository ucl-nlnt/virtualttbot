from setuptools import setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabriel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		#'sensors = sensors.sensors:main',
		'capstone = sensors.capstone:main',
        'data_test = sensors.data_test:main'
		# 'mess_test = sensors.message_receive_test:main',
		# 'movement_prot = sensors.movement_prot:main',
        #'inference_prog = sensors.inference_prog:main',
        #'level_test = sensors.level_test:main',
        ],
    },
)
