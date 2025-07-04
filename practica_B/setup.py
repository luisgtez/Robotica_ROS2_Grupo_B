import os
from glob import glob

from setuptools import find_packages, setup

package_name = "practica_B"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools", "opencv-python", "matplotlib", "numpy"],
    zip_safe=True,
    maintainer="alumno",
    maintainer_email="alumno@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_recon = practica_B.person_recon:main",
            "camera_node = practica_B.camera_node:main",
            "speed_monitor = practica_B.speed_monitor:main",
        ],
    },
)
