from setuptools import find_packages, setup

package_name = 'TM5_Control'

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
    maintainer='aaron',
    maintainer_email='bsunai@connect.ust',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Command_Send_Sub  = TM5_Control.Command_Send_Sub:main',
            'Joint_Pos_Pub_UI  = TM5_Control.Joint_Pos_Pub_UI:main',
            'Forward_Kinematics  = TM5_Control.Forward_Kinematics:main',
            'Joint_Pub  = TM5_Control.Joint_Pub:main',
            'Inverse_Kinematics  = TM5_Control.Inverse_Kinematics:main',
            'TM5_Admittance_Controller  = TM5_Control.TM5_Admittance_Controller:main',
            'All_in_One  = TM5_Control.All_in_One:main',
            'Force_Skin_Pub  = TM5_Control.Force_Skin_Pub:main',
            'Force_Skin_Sub  = TM5_Control.Force_Skin_Sub:main',
            'Skin_Test  = TM5_Control.Skin_Test:main',
            'Muti_Admittance_Control  = TM5_Control.Muti_Admittance_Control:main',
            'Joint_Space_Admittance  = TM5_Control.Joint_Space_Admittance:main',
            'tcp_control  = TM5_Control.tcp_control:main',
            'pvt_command  = TM5_Control.pvt_command:main',
            'pvt_test  = TM5_Control.pvt_test:main',
            'pvt_admittance_controller  = TM5_Control.pvt_admittance_controller:main',
            # Joint_Space_Admittance
            

        ],
    },
)
