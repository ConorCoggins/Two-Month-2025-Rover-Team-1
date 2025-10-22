from setuptools import find_packages, setup 

package_name = 'rospackage'

setup(
    name='rospackage',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='conorwcoggins@protonmail.com',
    description='Package that includes 4 nodes: a listener and talker for each the pico/libre and the base station',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest', 
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = rospackage.my_node:main'
            'basestationlistener = rospackage.basestationlistner:main'
            'basestationtalker = rospackage.basestationtalker:main'
            'picolistener = rospackage.picolistener:main'
            'picotalker = rospackage.picotalker:main'
        ],
    },
)
