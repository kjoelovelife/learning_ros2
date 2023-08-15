import os
from setuptools import setup
from glob import glob

package_name = 'lesson6_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"move_turtlesim_server_exe = {package_name}.move_turtlesim_server:main",
            f"move_turtlesim_client_exe = {package_name}.move_turtlesim_client:main"
        ],
    },
)
