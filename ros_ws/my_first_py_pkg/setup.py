from setuptools import find_packages, setup

package_name = 'my_first_py_pkg'

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
    maintainer='mmabas77',
    maintainer_email='mmabas77@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node = my_first_py_pkg.my_first_node:main",
            "draw_circle = my_first_py_pkg.draw_circle:main",
            "pose_subscriber = my_first_py_pkg.pose_subscriber:main"
        ],
    },
)
