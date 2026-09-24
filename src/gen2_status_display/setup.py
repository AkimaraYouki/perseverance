from glob import glob
from setuptools import setup

package_name = 'gen2_status_display'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parksudo',
    description='ST7789 robot status screen',
    license='Proprietary',
    entry_points={'console_scripts': [
        'status_display = gen2_status_display.status_display_node:main',
    ]},
)
