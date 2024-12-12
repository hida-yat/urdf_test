from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'urdf_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name), glob('launch/*.py')),
  	    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/my_room'), glob('models/my_room/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='okarobo',
    maintainer_email='okarobo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = urdf_test.robot_state_publisher:main'

        ],
    },
)
