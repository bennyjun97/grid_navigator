from setuptools import setup

package_name = 'grid_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyungbin',
    maintainer_email='hyungbin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pathway_detector = grid_navigator.pathway_detector_function:main',
                'navigator = grid_navigator.navigator_function:main',
        ],
    },
)
