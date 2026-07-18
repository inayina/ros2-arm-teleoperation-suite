from setuptools import find_packages, setup


package_name = 'isaac_sim_adapter'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
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
        ],
    },
)
