from setuptools import find_packages, setup


package_name = 'isaac_sim_adapter'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            's4_runtime_contract.json',
            's4_runtime_contract.yaml',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/scripts', ['scripts/isaac_panda_backend.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ina',
    maintainer_email='ina@example.com',
    description=(
        'ROS-only adapter from isolated Isaac Sim topics to the simulation contract.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_sim_adapter = isaac_sim_adapter.adapter_node:main',
            'isaac_e1_action_sequence = isaac_sim_adapter.e1_action_sequence:main',
            'isaac_policy_inference = isaac_sim_adapter.policy_inference_node:main',
            'isaac_smolvla_policy_inference = isaac_sim_adapter.smolvla_policy_inference_node:main',
        ],
    },
)
