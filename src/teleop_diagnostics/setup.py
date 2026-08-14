from setuptools import find_packages, setup

package_name = "teleop_diagnostics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/diagnostics.yaml"]),
        ("share/" + package_name + "/config", ["config/controller_reference_frame_contract.yaml"]),
        ("share/" + package_name + "/config", ["config/stage2_faults.yaml"]),
        ("share/" + package_name + "/launch", ["launch/geometry_diagnostics.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/geometry_live_tf.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Ina",
    maintainer_email="ina@example.com",
    description="Observer-only TF/FK geometry diagnostics.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "geometry_consistency_report = teleop_diagnostics.geometry_cli:main",
            "geometry_diagnostics_node = teleop_diagnostics.geometry_diagnostics_node:main",
            "geometry_live_tf_report = teleop_diagnostics.stage1_live_cli:main",
            "geometry_stage2_report = teleop_diagnostics.stage2_cli:main",
            "camera_stage3a_report = teleop_diagnostics.stage3a_cli:main",
            "camera_stage3b_report = teleop_diagnostics.stage3b_cli:main",
            "camera_stage3c_report = teleop_diagnostics.stage3c_cli:main",
            "timestamp_stage4_report = teleop_diagnostics.stage4_cli:main",
            "wrist_rgb_report = teleop_diagnostics.wrist_rgb_cli:main",
        ],
    },
)
