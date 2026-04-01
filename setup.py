package_name = 'robot_core'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot core package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = robot_core.serial_bridge:main',
            'robot_logic = robot_core.robot_logic:main',
            'serial_bridge_debug = robot_core.debugging.serial_bridge_debug:main'
        ],
    },
)
