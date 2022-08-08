from setuptools import setup

package_name = 'cerulean_sonar'
package_sub = 'brping'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_sub],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Rivera',
    maintainer_email='jackarivera@gmail.com',
    description='ROS2 Package to interface with cerulean sonar products',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = cerulean_sonar.sonar_node:main'
        ],
    },
)
