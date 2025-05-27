from setuptools import find_packages, setup

package_name = 'insperbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='rogeriobcuenca@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'menu = insperbot.menu:main',
            'servo_arm = insperbot.servo_arm:main',
            'april_tag = insperbot.april_tag:main',
        ],
    },
)
