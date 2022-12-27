import launch
import launch_ros.actions
import os
#cd /home/user/carlaRA/carla/CARLA_Shipping_0.9.11-dirty/LinuxNoEditor
#to run carla ./CarlaUE4.sh -quality-level=Low

def generate_launch_description():
    ld = launch.LaunchDescription()


    nodeObject=launch_ros.actions.Node(
            package='object_simulator',
            executable='object_simulator',
            parameters=[
            {"object_simulator_HOST": '127.0.0.1'},
            {'object_simulator_PORT': 2000},
           

            {'object_simulator_connectionTimeoutS': 2.0},
            {'object_simulator_objectFirst_tx': 0.0},
            {'object_simulator_objectFirst_ty': 0.0},
            {'object_simulator_objectFirst_tz': 3.0},
            {'object_simulator_objectFirst_rx': 0.0},
            {'object_simulator_objectFirst_ry': 0.0},
            {'object_simulator_objectFirst_rz': 0.0},
            {'object_simulator_objectBluePrint': "static.prop.palette"},
            
            {'object_simulator_disableRendering': 1},
            {'object_simulator_simulationTimeInS': 0.1}, #piste pour avoir du synchrone https://carla.readthedocs.io/en/0.9.11/adv_synchrony_timestep/
            ],
            name='object_simulator',
            output='screen',
            remappings=[
                #("robot_simulator_cmd_vel", "robot_sender_cmd_vel")
                #("/robot_sender_poseInUtmTiles", "/robot_simulator_poseInUtmTiles")
            ]
            )
    ld.add_action(nodeObject)


    nodeLidar=launch_ros.actions.Node(
            package='lidar_simulator',
            executable='lidar_simulator',
            parameters=[
            {"lidar_simulator_HOST": '127.0.0.1'},
            {'lidar_simulator_PORT': 2000},
            {'lidar_simulator_LIDAR_MAX_RANGE_IN_M': 50.0},
            {'lidar_simulator_LIDAR_NB_LAYERS': 16},
            {'lidar_simulator_LIDAR_ROTATION_FREQUENCY_HZ': 10.0},
            {'lidar_simulator_LIDAR_UPPER_FOV_IN_DEG': 15},
            {'lidar_simulator_LIDAR_LOWER_FOV_IN_DEG': -16},
            {'lidar_simulator_noise_stddev': 0.02},

            {'lidar_simulator_points_per_second': 10000},#2000

            {'lidar_simulator_connectionTimeoutS': 2.0},
            {'lidar_simulator_lidarFirst_tx': 0.0},
            {'lidar_simulator_lidarFirst_ty': 0.0},
            {'lidar_simulator_lidarFirst_tz': 3.0},
            {'lidar_simulator_lidarFirst_rx': 0.0},
            {'lidar_simulator_lidarFirst_ry': 0.0},
            {'lidar_simulator_lidarFirst_rz': 0.0},
            {'lidar_simulator_disableRendering': 1},
            {'lidar_simulator_simulationTimeInS': 0.1}, #piste pour avoir du synchrone https://carla.readthedocs.io/en/0.9.11/adv_synchrony_timestep/
            ],
            name='lidar_simulator',
            output='screen',
            remappings=[
                ("/lidar_simulator_lidarPose", "/object_simulator_poseObjectTwist")
                #("robot_simulator_cmd_vel", "robot_sender_cmd_vel")
                #("/robot_sender_poseInUtmTiles", "/robot_simulator_poseInUtmTiles")
            ]
            )
    ld.add_action(nodeLidar)


    nodeCamera=launch_ros.actions.Node(
            package='camera_simulator',
            executable='camera_simulator',
            parameters=[
            {"camera_simulator_HOST": '127.0.0.1'},
            {'camera_simulator_PORT': 2000},
            {'camera_simulator_WIDTH': 1280},
            {'camera_simulator_HEIGHT': 720},
            {'camera_simulator_FOV': 69.0},
            {'camera_simulator_connectionTimeoutS': 2.0},
            {'camera_simulator_cameraFirst_tx': 0.0},
            {'camera_simulator_cameraFirst_ty': 0.0},
            {'camera_simulator_cameraFirst_tz': 3.0},
            {'camera_simulator_cameraFirst_rx': 0.0},
            {'camera_simulator_cameraFirst_ry': 0.0},
            {'camera_simulator_cameraFirst_rz': 0.0},
            {'camera_simulator_disableRendering': 1},
            {'camera_simulator_simulationTimeInS': 0.1}, #piste pour avoir du synchrone https://carla.readthedocs.io/en/0.9.11/adv_synchrony_timestep/
            ],
            name='camera_simulator',
            output='screen',
            remappings=[
                ("/camera_simulator_cameraPose", "/lidar_simulator_poseLidarTwist")
                #("robot_simulator_cmd_vel", "robot_sender_cmd_vel")
                #("/robot_sender_poseInUtmTiles", "/robot_simulator_poseInUtmTiles")
            ]
            )
    ld.add_action(nodeCamera)


    node2=launch_ros.actions.Node(
            package='robot_simulator',
            executable='robot_simulator',
            parameters=[
            {"robot_simulator_defaultLatitude": 49.383224},
            {'robot_simulator_defaultLongitude': 1.073758},
            {'robot_simulator_defaultYawDeg': 0.0}
            ],
            name='robot_simulator',
            output='screen',
            remappings=[
                #("robot_simulator_cmd_vel", "robot_sender_cmd_vel")
                #("/robot_sender_poseInUtmTiles", "/robot_simulator_poseInUtmTiles")
            ]
            )
    #ld.add_action(node2)

    #lancement du teleop dans un sous shell
    #sudo apt install xterm si besoin

    nodeTurtle=launch_ros.actions.Node( 
                                    package='turtlesim', 
                                    executable='turtle_teleop_key', 
                                    output='screen',
                                    prefix=['xterm -e'],
                                    remappings=[("/turtle1/cmd_vel", "/camera_simulator_cmd_vel")]
                                    #remappings=[("/turtle1/cmd_vel", "/lidar_simulator_cmd_vel")]
                                    
                                    
                                    #remappings=[("/robot_simulator_cmd_vel","/turtle1/cmd_vel")]
                                    )
    ld.add_action(nodeTurtle)

    #lancement d'un viewer de topic

    #display topic
    #os.popen("xterm -e \"source /opt/ros/rolling/setup.bash;ros2 topic echo /robot_simulator_poseInUtmTiles\"") 
    os.popen("xterm -e \"source /opt/ros/rolling/setup.bash;ros2 topic echo /camera_simulator_cameraPose\"") 


    #rviz display
    noderviz=launch_ros.actions.Node( 
                                    package='rviz2', 
                                    executable='rviz2', 
                                    output='screen',
                                    arguments=['-d', "./ros2/configRviz.rviz"]
                                    )
    #ld.add_action(noderviz)


    """

     executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        """

    #turtlesim turtle_teleop_key
    return ld
    """
    node1=launch_ros.actions.Node(
            package='colibry_communicator',
            executable='robot_sender',
            parameters=[
            {'robot_sender_FREQUENCY': 10},
            {'robot_sender_robotName': "Fab1(MIR)"},
            {'robot_sender_ENCRYPT': 1},
            #{'robot_sender_url': "opc.tcp://127.0.0.1:4840/freeopcua/server/"},
            {'robot_sender_url': "opc.tcp://esigelec.ddns.net:11111/freeopcua/server/"},
            #{'robot_sender_url': "opc.tcp://192.168.2.105:4840/freeopcua/server/"},
            {'robot_sender_namespace': "http://esigelec.ddns.net"},
            #{'robot_sender_certificate': "vincent/my_cert.der"},
            #{'robot_sender_private_key': "vincent/my_private_key.pem"},
            {'robot_sender_certificate': "/home/user/colibry/opcuaRt/MIR1/my_cert.der"},
            {'robot_sender_private_key': "/home/user/colibry/opcuaRt/MIR1/my_private_key.pem"},
            {'robot_sender_defaultLatitude': 49.383224},
            {'robot_sender_defaultLongitude': 1.073758}
            ],
            name='robot_sender',
            output='screen',
            remappings=[
                #("robot_simulator_cmd_vel", "robot_sender_cmd_vel")
                #("/robot_sender_poseInUtmTiles", "/robot_simulator_poseInUtmTiles")
            ]
            )
    ld.add_action(node1)

 


    node2= launch_ros.actions.Node(
            package='colibry_robot_simulator',
            executable='robot_simulator',
            parameters=[
            {"robot_simulator_defaultLatitude": 49.383224},
            {'robot_simulator_defaultLongitude': 1.073758},
            {'robot_simulator_defaultYawDeg': 0.0}
            ],
            name='robot_simulator',
            output='screen',
            remappings=[
                #/turtle1/cmd_vel
                #("robot_simulator_poseInUtmTiles", "robot_sender_poseInUtmTiles")
                ("/robot_simulator_cmd_vel", "/robot_sender_cmd_vel"),
                #("/robot_simulator_cmd_vel", "/turtle1/cmd_vel"),#turle move
                ("/robot_simulator_poseInUtmTiles", "/robot_sender_poseInUtmTiles")
            ]
            )
    ld.add_action(node2)
    return ld
    """
    """return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='colibry_communicator',
            executable='robot_sender',
            name='robot_sender',
            output='screen',
            parameters=[
            {'toto': 10},
            {'robot_sender_FREQUENCY': 10},
            {'robot_sender_robotName': "Fab1(MIR)"},
            {'robot_sender_ENCRYPT': 0},
            {'robot_sender_url': "opc.tcp://127.0.0.1:4840/freeopcua/server/"},
            {'robot_sender_namespace': "http://esigelec.ddns.net"},
            {'robot_sender_certificate': "vincent/my_cert.der"},
            {'robot_sender_private_key': "vincent/my_private_key.pem"},
            {'robot_sender_defaultLatitude': 49.383224},
            {'robot_sender_defaultLongitude': 1.073758}
            ]
            ),
        launch_ros.actions.Node(
            package='colibry_robot_simulator',
            executable='robot_simulator',
            name='robot_simulator',
            output='screen'
            ),
    ])"""
#ros2 run colibry_robot_simulator robot_simulator
#ros2 run colibry_communicator robot_sender