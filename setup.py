import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'go1_bringup'
map_files = [path for path in glob('maps/*') if os.path.isfile(path)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        # 只安装 maps 下的普通文件; maps/sessions/ 是大体积归档目录, 由 .gitignore 排除.
        (os.path.join('share', package_name, 'maps'), map_files),
    ],
    # bash 评估脚本装到 <prefix>/lib/<pkg>/, 让 ros2 run 可直接调用.
    # python 脚本已迁移到包内并通过 console_scripts 注册, 不再走 share 路径.
    scripts=['scripts/evaluate_slam.sh'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ziggy',
    maintainer_email='yudongliu.bit@gmail.com',
    description='Go1 robot bringup package for autonomous navigation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # 所有 python 评估/工具脚本统一通过 ros2 run go1_bringup <name> 调用,
        # 跨主机/跨工作区不再依赖绝对源码路径 (此前手册里到处是 python3 ~/go1_ws/src/...).
        'console_scripts': [
            'archive_map = go1_bringup.archive_map:main',
            'pcd_to_map = go1_bringup.pcd_to_map:main',
            'record_trajectory = go1_bringup.record_trajectory:main',
            'nav_metrics = go1_bringup.nav_metrics:main',
            'measure_geometry = go1_bringup.measure_geometry:main',
            'experiment_log = go1_bringup.experiment_log:main',
            'threshold_sensitivity = go1_bringup.threshold_sensitivity:main',
        ],
    },
)
