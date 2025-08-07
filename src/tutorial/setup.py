from setuptools import setup

package_name = 'tutorial'

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
    maintainer='user',
    maintainer_email='gunx9@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"gss_test = tutorial.gss_test:main",
        	"control_test = tutorial.control_test:main",
        	"camera_test = tutorial.camera_test:main",
        	"lidar_test = tutorial.lidar_test:main"
        ],
    },
)
