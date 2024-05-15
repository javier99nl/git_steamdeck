from setuptools import setup

package_name = 'bluerov2_teleop'

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
    maintainer='tijmen',
    maintainer_email='t.vanenckevort@gmail.com',
    description='bluerov2_teleop',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'joy_teleop = bluerov2_teleop.bluerov2_teleop:main'
        ],
    },
)
