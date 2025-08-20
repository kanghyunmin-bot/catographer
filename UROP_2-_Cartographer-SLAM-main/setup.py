from setuptools import setup
from ament_index_python.packages import get_package_share_directory

package_name = 'a1m8_cartographer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 루트 launch/ 와 params/를 설치 경로로 복사
        ('share/' + package_name + '/launch', ['launch/slam_a1m8.launch.py']),
        ('share/' + package_name + '/params', [
            'params/rplidar_a1m8.yaml',
            'params/occupancy_grid.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='A1M8 + Cartographer 2D SLAM with runtime-generated Lua',
    license='Apache-2.0',
)
