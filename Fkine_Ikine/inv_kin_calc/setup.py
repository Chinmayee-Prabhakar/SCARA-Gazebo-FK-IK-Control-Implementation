from setuptools import setup

package_name = 'inv_kin_calc'

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
    maintainer='sohamaserkar',
    maintainer_email='sohamaserkar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service = inv_kin_calc.scara_inverse_server:main',
        'client = inv_kin_calc.scara_inverse_client:main',
        ],
    },
)
