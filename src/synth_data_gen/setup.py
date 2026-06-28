import os
from glob import glob

from setuptools import find_packages, setup

package_name = "synth_data_gen"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ina",
    maintainer_email="ina@example.com",
    description="Batch synthetic data generator for LeRobot training.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "batch_generator = synth_data_gen.batch_generator:main",
        ],
    },
)
