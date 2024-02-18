from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gr_kinematic_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leev',
    maintainer_email='yyootttaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = gr_kinematic_sim.main:main'
        ],
    },
    py_modules=[
        package_name + '.custom_utils.collisions',
        package_name + '.custom_utils.gametools',
        package_name + '.custom_utils.mathtools',
        package_name + '.custom_utils.object_tools',
        package_name + '.custom_utils.sensors',
        package_name + '.custom_utils.robots',
        package_name + '.custom_utils.rostools',
    ],
)
