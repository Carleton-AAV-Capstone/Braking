from setuptools import find_packages, setup

package_name = 'aav_braking'

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
    maintainer='msaud',
    maintainer_email='immsaud@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    #executable_name = package.file_name
    entry_points={
        'console_scripts': [
            "braking_pub_exe = aav_braking.braking_pub:main",
            "braking_sub_exe = aav_braking.braking_sub:main"
        ],
    },
)
