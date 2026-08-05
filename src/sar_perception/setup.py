from setuptools import setup

package_name = 'sar_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raccoon',
    maintainer_email='raccoon@example.com',
    description='Detection and geotagging nodes for the SAR grid explorer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detection_node = sar_perception.detection_node:main',
            'geotag_node = sar_perception.geotag_node:main',
        ],
    },
)
