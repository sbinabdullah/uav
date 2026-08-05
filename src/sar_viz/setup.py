from setuptools import setup

package_name = 'sar_viz'

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
    description='Telemetry and map visualization for the SAR grid explorer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'telemetry_viz_node = sar_viz.telemetry_viz_node:main',
        ],
    },
)
