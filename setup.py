from setuptools import setup

package_name = 'rig_reconfigure'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Spatzenhirn',
    maintainer_email='team-spatzenhirn@uni-ulm.de',
    description='Dear ImGui based alternative for RQT reconfigure plugin',
    license='MIT',
    entry_points={
        'console_scripts': [
            'reconfigurator = rig_reconfigure.rig_reconfigure:main',
        ],
    },
)
