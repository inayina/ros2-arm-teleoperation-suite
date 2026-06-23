from setuptools import find_packages, setup

package_name = "virtual_servo_driver"

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
    description="L4 CANopen DS402 virtual servo drive.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "virtual_servo_driver = virtual_servo_driver.driver_node:main",
        ],
    },
)
