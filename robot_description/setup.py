from setuptools import setup, find_packages
from glob import glob
import os

package_name = "robot_description"

setup(
    name=package_name,
    packages=find_packages(),
    install_requires=[
        "setuptools",
        "launch",
        "launch_ros"
    ],
    package_data={
        package_name: [
            "meshes/*.STL",
            "*.urdf"
        ]
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.STL"))
    ],
    entry_points={
        "console_scripts": []
    },
    zip_safe=True
)
