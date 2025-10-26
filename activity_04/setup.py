from setuptools import find_packages, setup

package_name = 'activity_04'

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
    maintainer='aashishkumar',
    maintainer_email='aashishkumar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "led_panel_node = activity_04.led_panel_node:main",
            "battery_node = activity_04.battery_node:main"
        ],
    },
)
