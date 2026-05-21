from setuptools import find_packages, setup

package_name = 'plastic_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['camera_to_robot_calibration.json', 'best.pt'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rian',
    maintainer_email='edwardsrian8@gmail.com',
    description='OBB plastic detection node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obb_detector_node = plastic_detection.obb_detector_node:main',
            'grip_pose_node = plastic_detection.grip_pose_node:main',
            'obb_detector_bottom_point = plastic_detection.obb_detector_bottom_point:main',
            'obb_detector_tracked = plastic_detection.obb_detector_tracked:main',
            'obb_detector_unified_plane = plastic_detection.obb_detector_unified_plane:main',
            'obb_detector_homography = plastic_detection.obb_detector_homography:main',
            'obb_detector_improved = plastic_detection.obb_detector_improved:main',
        ],
    },
)
