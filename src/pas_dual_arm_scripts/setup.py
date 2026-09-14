from setuptools import find_packages, setup

package_name = 'pas_dual_arm_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khartl',
    maintainer_email='kh239762@fsb.hr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector = pas_dual_arm_scripts.aruco_detector:main',
            'cmd_vel_relay = pas_dual_arm_scripts.cmd_vel_relay:main',
            'main_task = pas_dual_arm_scripts.main_task:main',
            'mapping_tour = pas_dual_arm_scripts.mapping_tour:main',
            'set_posture = pas_dual_arm_scripts.set_posture:main',
            'table_ready = pas_dual_arm_scripts.table_ready:main',
            'scan_filter = pas_dual_arm_scripts.scan_filter:main',
            'footprint_publisher = pas_dual_arm_scripts.footprint_publisher:main',
            'feature_registry = pas_dual_arm_scripts.feature_registry:main',
            'set_camera = pas_dual_arm_scripts.set_camera:main',
            'nav_zones = pas_dual_arm_scripts.nav_zones:main',
            'room_navigator = pas_dual_arm_scripts.room_navigator:main',
            'nav_gui = pas_dual_arm_scripts.nav_gui:main',
            'loc_error = pas_dual_arm_scripts.loc_error:main',
        ],
    },
)
