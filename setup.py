from setuptools import setup

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Wallace',
    maintainer_email='wallace.teixeira@unicamperacing.com.br',
    description='Integrando Jetson com steppermotor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simula_planning_node = vehicle_control.simula_planning_node:main',
            'control_node = vehicle_control.control_node:main',
        ],
    },
)
