from setuptools import find_packages, setup

package_name = 'task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),(f'{package_name}/steering_visualization.py')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farraj',
    maintainer_email='BARHAMFARRAJ@ICLOUD.COM',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'task=task.steering_visualization:main'],
    },
)
