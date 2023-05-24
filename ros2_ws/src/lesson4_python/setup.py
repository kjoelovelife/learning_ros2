import os
from setuptools import setup
from glob import glob

package_name = 'lesson4_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", f"{package_name}/launch"), glob("launch/*.launch.py")),
        (os.path.join("share", f"{package_name}/launch"), glob("launch/tutorial_answer/*.launch.py")),
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
            f"publisher  = {package_name}.publisher:main",
            f"subscriber = {package_name}.subscriber:main",
            f"turtle_draw_circle_answer = {package_name}.tutorial_answer.turtle_draw_circle_answer:main",
            ### === modify code below ===
            f"",
            ### === end ===
        ],
    },
)
