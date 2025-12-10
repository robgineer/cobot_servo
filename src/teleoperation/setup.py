from setuptools import setup
from glob import glob

package_name = "teleoperation"

setup(
    name=package_name,
    packages=[package_name],
    version="1.0.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Harbach",
    maintainer_email="robgineer@gmail.com",
    description="Teleoperation using MoveIt2 Servo",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": ["hand_control = teleoperation.hand_control:main"],
    },
)
