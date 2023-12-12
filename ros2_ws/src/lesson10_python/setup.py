import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'lesson10_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brogent',
    maintainer_email='brogent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"retrieve_laser_scan_exe = {package_name}.retrieve_laser_scan:main"
        ],
    },
)
