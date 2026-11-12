from setuptools import setup

package_name = 'rm_nav_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/field.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/sim.launch.py',
            'launch/nav.launch.py',
            'launch/nav_truth.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RM Vision Team',
    maintainer_email='dev@example.com',
    description='Lightweight 2D grid navigation simulator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_node = rm_nav_sim.sim_node:main',
            'gen_map = rm_nav_sim.gen_map:main',
        ],
    },
)
