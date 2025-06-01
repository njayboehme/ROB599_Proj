from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vlm_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'geometry_msgs',
        'nav_game_msgs',
        'nav_game'
    ],
    zip_safe=True,
    maintainer='nboehme',
    maintainer_email='boehmen@oregonstate.edu',
    description='ROB599 VLM Motion Planner Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm = vlm_nav.vlm:main',
            'text_client = vlm_nav.text_client:main',
            'camera_client = vlm_nav.camera_client:main',
            'problem_manager = vlm_nav.problem_manager:main'
        ],
    },
)
