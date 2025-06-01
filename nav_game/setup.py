from setuptools import setup

package_name = 'nav_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'rclpy',
        'nav_game_msgs',
        'geometry_msgs',
        'pygame'
    ],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='ROS 2 action server that runs a minimal pygame grid game',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_game = nav_game.nav_game:main'
        ],
    },
)
