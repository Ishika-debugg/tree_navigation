from setuptools import setup, find_packages

package_name = 'tree_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Tree detection and coordinated navigation for ROSBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tree_detector = tree_navigation.tree_detector:main',
            'brown_navigator = tree_navigation.brown_color_navigator:main',
            'tree_navigation_coordinator = tree_navigation.tree_navigation_coordinator:main',
        ],
    },
)
