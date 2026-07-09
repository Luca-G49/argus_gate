from setuptools import find_packages, setup

package_name = 'argus_flet_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools', 'flet'],
    zip_safe=True,
    maintainer='Luca',
    maintainer_email='luca.gherardo49@gmail.com',
    description='Flet GUI for Argus Gate manager',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'argus_flet_gui = argus_flet_gui.gui:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/flet_gui.launch.py']),
    ],
    include_package_data=True,
)
