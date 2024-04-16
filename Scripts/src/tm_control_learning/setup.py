from setuptools import find_packages, setup

package_name = 'tm_control_learning'

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
            'Ask_Item  = tm_control_learning.Ask_Item:main',
            'Ask_Sta  = tm_control_learning.Ask_Sta:main',
            'Connect_Tm  = tm_control_learning.Connect_Tm:main',
            'Get_Feedback  = tm_control_learning.Get_Feedback:main',
            'Get_Sct_Response  = tm_control_learning.Get_Sct_Response:main',
            'Get_Sta_Response  = tm_control_learning.Get_Sta_Response:main',
            'Get_Status  = tm_control_learning.Get_Status:main',
            'Get_Svr_Response  = tm_control_learning.Get_Svr_Response:main',
            'Get_Torque_Feedback  = tm_control_learning.Get_Torque_Feedback:main',
            'Leave_Listen_Node  = tm_control_learning.Leave_Listen_Node:main',
            'Script_Send  = tm_control_learning.Script_Send:main',
            'Send_Command  = tm_control_learning.Send_Command:main',
            'Set_Event  = tm_control_learning.Set_Event:main',
            'Set_IO  = tm_control_learning.Set_IO:main',
            'Set_Position  = tm_control_learning.Set_Position:main',
            'Write_Item  = tm_control_learning.Write_Item:main',


        ],
    },
)
