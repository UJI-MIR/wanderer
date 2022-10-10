from setuptools import setup

package_name = 'exercises'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vipul',
    maintainer_email='garg.vipul7@gmail.com',
    description='Multi Robot Exercises',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_sub = exercises.odom_listener:main',
            'scan_sub = exercises.scan_listener:main',
            'cmd_pub = exercises.cmd_publisher:main',
            'move_forward = exercises.move_forward:main',
            'wanderer = exercises.wanderer:main',
        ],
    },
)
