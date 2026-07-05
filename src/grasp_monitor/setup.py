import os
from glob import glob

from setuptools import find_packages, setup

package_name = "grasp_monitor"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ina",
    maintainer_email="ina@example.com",
    description="Passive M7 grasp state monitor and advisor.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_monitor_node = grasp_monitor.grasp_monitor_node:main",
        ],
    },
)
