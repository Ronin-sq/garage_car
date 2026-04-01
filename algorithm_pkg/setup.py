from setuptools import find_packages, setup
import os
import glob

package_name = 'algorithm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ronin',
    maintainer_email='2097802563@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_node = algorithm_pkg.scan_node:main',
            'action_server = algorithm_pkg.action_server:main',
            'action_client = algorithm_pkg.action_client:main',
            'nav_path_node = algorithm_pkg.nav_path_node:main',
            'rebar_path_executor = algorithm_pkg.rebar_path_executor:main',
        ],
    },
)
