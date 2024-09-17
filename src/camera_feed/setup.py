from setuptools import find_packages, setup

package_name = 'camera_feed'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gregory',
    maintainer_email='gregoryarlanda95@gmail.com',
    description='Camera feed package for streaming IP camera feeds',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream = camera_feed.camera_stream:main',
        ],
    },
)

