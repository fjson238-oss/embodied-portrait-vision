from setuptools import find_packages, setup

package_name = 'vision_skeleton'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [
            'models/pose_landmarker_lite.task',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fj',
    maintainer_email='fj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'skeleton_node = vision_skeleton.skeleton_node:main',
        ],
    },
)
