from glob import glob

from setuptools import setup

package_name = 'tunibot_delivery'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ghassen Bani',
    maintainer_email='ghassen.bani@supcom.tn',
    description='TuniBot autonomous restaurant delivery robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'delivery_manager = tunibot_delivery.delivery_manager:main',
            'web_api = tunibot_delivery.web_api:main',
        ],
    },
)
