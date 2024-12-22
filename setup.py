from setuptools import setup

package_name = 'rqt_bag_ffmpeg_plugin'
setup(
    name=package_name,
    version='0.0.1',
    package_dir={'': 'src'},
    packages=['rqt_bag_ffmpeg_plugin'],
    data_files=[
        ('share/' + package_name, ['plugin.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@example.com',
    keywords=['ROS'],
    description=(
        'rqt_bag_ffmpeg_plugin provides GUI plugins for rqt_bag to display ffmpeg_image_transport_msgs/msg/FFMPEGPacket.'
    ),
    license='BSD',
)
