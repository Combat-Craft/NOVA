from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/joystick.launch.py', 'launch/motor.launch.py']),
        ('share/' + package_name + '/config', ['config/joystick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rida-e',
    maintainer_email='rida-e@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = controller.joystick:main',
            'motor= controller.motor:main',
            'motor_driver = controller.motor_driver:main', # This points to the joystick.py file
            'joystick_to_motor = controller.joystick_to_motor:main'
        ],
    },
)
