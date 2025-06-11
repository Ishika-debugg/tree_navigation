from setuptools import setup

package_name = 'tree_navigation'

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
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Waypoint navigation for macadamia farm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = tree_navigation.waypoint_navigator:main',
        ],
    },
)
