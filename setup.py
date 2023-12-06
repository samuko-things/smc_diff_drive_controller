from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smc_diff_drive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name, [package_name+'/modules/smc_arduino_pyserial_comm.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'smc_diff_drive_controller = smc_diff_drive_controller.smc_diff_drive_controller:main',
        ],
    },
)