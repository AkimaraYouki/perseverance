from glob import glob
from setuptools import setup

setup(
    name='gen2_tools',
    version='0.1.0',
    packages=['gen2_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gen2_tools']),
        ('share/gen2_tools', ['package.xml']),
        ('share/gen2_tools/scripts', glob('scripts/*')),
    ],
    scripts=['scripts/record_bag.sh', 'scripts/install_desktop_shortcuts.sh'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parksudo',
    description='Gen2 bench tools',
    license='Proprietary',
    entry_points={'console_scripts': [
        'hub_cli = gen2_tools.hub_cli:main',
        'motor_test_gui = gen2_tools.motor_test_gui:main',
        'bag_to_csv = gen2_tools.bag_to_csv:main',
    ]},
)
