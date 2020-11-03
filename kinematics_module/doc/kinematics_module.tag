<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>global_configuration.h</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/kinematics/</path>
    <filename>global__configuration_8h</filename>
    <class kind="class">GlobalConfiguration</class>
  </compound>
  <compound kind="file">
    <name>kinematics.cpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/kinematics/</path>
    <filename>kinematics_8cpp</filename>
    <includes id="kinematics_8h" name="kinematics.h" local="yes" imported="no">kinematics.h</includes>
    <includes id="global__configuration_8h" name="global_configuration.h" local="yes" imported="no">global_configuration.h</includes>
  </compound>
  <compound kind="file">
    <name>kinematics.h</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/kinematics/</path>
    <filename>kinematics_8h</filename>
    <class kind="class">KinematicSolver</class>
    <member kind="function">
      <type>int</type>
      <name>sgn</name>
      <anchorfile>kinematics_8h.html</anchorfile>
      <anchor>a1ab31b90bc584c635ec159468ceed9b2</anchor>
      <arglist>(T val)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>nullspace_generator.cpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/nullspace/</path>
    <filename>nullspace__generator_8cpp</filename>
    <includes id="nullspace__generator_8hpp" name="nullspace_generator.hpp" local="yes" imported="no">nullspace_generator.hpp</includes>
    <member kind="define">
      <type>#define</type>
      <name>_USE_MATH_DEFINES</name>
      <anchorfile>nullspace__generator_8cpp.html</anchorfile>
      <anchor>a525335710b53cb064ca56b936120431e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>nullspace_generator.hpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/nullspace/</path>
    <filename>nullspace__generator_8hpp</filename>
    <class kind="class">NullspaceGenerator</class>
    <member kind="define">
      <type>#define</type>
      <name>EMPTY_INTERVAL_BORDER</name>
      <anchorfile>nullspace__generator_8hpp.html</anchorfile>
      <anchor>aada32eef6ee71e97478a583c25ac266e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>IS_EMPTY_INTERVAL_BORDER</name>
      <anchorfile>nullspace__generator_8hpp.html</anchorfile>
      <anchor>a3d819a5af196130209e73bb30cd703b8</anchor>
      <arglist>(val)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>nullspace_solver.cpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/nullspace/</path>
    <filename>nullspace__solver_8cpp</filename>
    <includes id="nullspace__solver_8hpp" name="nullspace_solver.hpp" local="yes" imported="no">nullspace_solver.hpp</includes>
    <member kind="define">
      <type>#define</type>
      <name>_USE_MATH_DEFINES</name>
      <anchorfile>nullspace__solver_8cpp.html</anchorfile>
      <anchor>a525335710b53cb064ca56b936120431e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>EXT_DEBUG</name>
      <anchorfile>nullspace__solver_8cpp.html</anchorfile>
      <anchor>a7ea5444b741b56dab9edcb4d0f5040b9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>nullspace_solver.hpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/nullspace/</path>
    <filename>nullspace__solver_8hpp</filename>
    <class kind="class">NullspaceSolver</class>
    <member kind="function">
      <type>int</type>
      <name>sgn</name>
      <anchorfile>nullspace__solver_8hpp.html</anchorfile>
      <anchor>a1ab31b90bc584c635ec159468ceed9b2</anchor>
      <arglist>(T val)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>kinematics_module-component.cpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/src/</path>
    <filename>kinematics__module-component_8cpp</filename>
    <includes id="kinematics__module-component_8hpp" name="kinematics_module-component.hpp" local="yes" imported="no">kinematics_module-component.hpp</includes>
    <member kind="define">
      <type>#define</type>
      <name>_USE_MATH_DEFINES</name>
      <anchorfile>kinematics__module-component_8cpp.html</anchorfile>
      <anchor>a525335710b53cb064ca56b936120431e</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" static="yes">
      <type>static const string</type>
      <name>PROP_FILE</name>
      <anchorfile>kinematics__module-component_8cpp.html</anchorfile>
      <anchor>ae8cfa37e85eb6adea2bc5997241837d7</anchor>
      <arglist>(&quot;/home/carlos/Workspace/src/Properties/KUKA_LBR_IIWA.xml&quot;)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>kinematics_module-component.hpp</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/src/</path>
    <filename>kinematics__module-component_8hpp</filename>
    <includes id="kinematics_8h" name="kinematics.h" local="yes" imported="no">kinematics.h</includes>
    <class kind="class">KinematicsModule</class>
  </compound>
  <compound kind="class">
    <name>GlobalConfiguration</name>
    <filename>classGlobalConfiguration.html</filename>
    <member kind="function">
      <type></type>
      <name>GlobalConfiguration</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>a112f0f38a5afe5fa7f6b42d1fe156a6d</anchor>
      <arglist>(unsigned int rconf)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>arm</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>a7a9f7cc7dbb6de89a528e192cb7a49de</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>elbow</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>a101912794711bc037addaa87979dd6d4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>wrist</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>ab4c76c527d1222f5f3d7e14c87ae1212</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>int</type>
      <name>m_arm</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>a1fd5a5c8fa54719b5b550219e4e74b68</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>int</type>
      <name>m_elbow</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>aa03188702e6a790363bffa6e602f0b09</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>int</type>
      <name>m_wrist</name>
      <anchorfile>classGlobalConfiguration.html</anchorfile>
      <anchor>abcc5a380612d423fd24e4d7d227a2f08</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>KinematicsModule</name>
    <filename>classKinematicsModule.html</filename>
    <member kind="function">
      <type></type>
      <name>KinematicsModule</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a6fc4d63554278a7db3be59e0158dc532</anchor>
      <arglist>(const std::string &amp;component_name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>configureHook</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>afb8040a078533a8389679f29ab707a43</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>startHook</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>af5b84daf7219927edae7dc7166974653</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateHook</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aaa86e862e85572b409374562d23e9e13</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopHook</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a66c246d870446f04d9c56ccf3c724a4c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cleanupHook</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae41af4139b9eae48bb15bdb6b9266dee</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a95038cf9d24934a3b0dffb97ee957973</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>double</type>
      <name>m_tol</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a0ccb9389a7348c481aa1cfff9956bdc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_lim</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a65b7a749771c57e1eb56d04a5815c884</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_lnk</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae891cd9064a30e630f58f56e4fdc170f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_a</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ab790914a722c58a04da3bd6d8c8c6d33</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_alpha</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a41d61f38c205ebcce3c37b81ba6e3bcf</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_d</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a20472756e8805488c79fea96f58a51f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_theta</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aba61a3aced848d5e9c52e1598ea9a607</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; std::vector&lt; double &gt; &gt;</type>
      <name>m_dh</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aa16c855de19b8853840d67b82e9e6e4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; robot_msgs::Posture &gt;</type>
      <name>inport_current_robot_param</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aef741a39faabf471bb4c3dd3b57399ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>inport_current_joint_positions</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae736a6f3d9f188e39f94ee14ff445eb4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; KDL::Frame &gt;</type>
      <name>inport_online_frame</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a75d8ac3a55f8d24fe41fa99b664ab107</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>outport_online_joint_positions</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a1db553cec60eb0818579b2489078ffc7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; error_msgs::Error &gt;</type>
      <name>outport_error_msg</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a952ff6ee229527893301cfeef7ed6d23</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration" protection="private">
      <type></type>
      <name>KIN_MODE</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae3537bdd6145851fd8d1a4e9f6097071</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>TFC</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae3537bdd6145851fd8d1a4e9f6097071aed954503fff8a5c81af16172d2860dab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>OTG</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae3537bdd6145851fd8d1a4e9f6097071ac5cdc9184047867a13f2137c5b02cfac</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>SetOnlineTrajectoryGeneration</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aed2bd345d158e86b8c89d79370054f17</anchor>
      <arglist>(bool otg)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>int</type>
      <name>CalculateJointTrajectory</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aaf465edcb7dae3ab267fd4af0e3bcbc4</anchor>
      <arglist>(const std::vector&lt; KDL::Frame &gt; &amp;target_frames, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;output_joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>AnalyseTrajectory</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ae43b785f459abf825a8b6909816ecb0e</anchor>
      <arglist>(std::vector&lt; KDL::Frame &gt; &amp;target_frames, const double max_nschange, std::vector&lt; unsigned int &gt; &amp;possible, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;initial_interval)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>int</type>
      <name>ScoresTrajectory</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>aeb937c6036446f557d62eb3ee3659b01</anchor>
      <arglist>(std::vector&lt; unsigned int &gt; &amp;possible, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;initial_interval, const double max_nschange, int gc_alt[8], std::vector&lt; std::vector&lt; double &gt; &gt; &amp;scores)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>Selectgc</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a546f9429c3e6149e035a30a28aa767dd</anchor>
      <arglist>(std::vector&lt; std::vector&lt; double &gt; &gt; &amp;jconf, std::vector&lt; double &gt; &amp;previous_joints, std::vector&lt; double &gt; &amp;target_joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>GenerateSingleNSInterval</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a80bde3a343f43026816377e82e813039</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;ns_scores)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>CheckWaypointsInWorkspace</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a5c1e3da4283df0664ab0032ed31a6629</anchor>
      <arglist>(const std::vector&lt; KDL::Frame &gt; &amp;target_frames)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>CalculateFKSolution</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ab5e5b1d6b900ef0a2072aa40c4092a1e</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;joints, KDL::Frame &amp;frame, double &amp;nsparam, unsigned int &amp;gc)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>CalculateIKSolution</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a9c25293e9413d17c3a721e199608d098</anchor>
      <arglist>(const KDL::Frame target, const double nsparam, const unsigned int gc, std::vector&lt; double &gt; &amp;joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>PrintToFile</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a8e57a04764f0545955a3a934f9f120a4</anchor>
      <arglist>(std::string file_path)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; KinematicSolver &gt;</type>
      <name>m_kinematic_solver</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ab6da791e58e75fb4497256696b18d49a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_current_frame</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a1636a7de0e5d70a06d94876f5761c7b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_current_nsparam</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a7e7974ef6261e89cd898254f169cf343</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_current_gc</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a93d0fbbcaadc2c2a018a56d1f377c010</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_current_joint_positions</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>ad13d75d2a50dd6f4c5cb955b8e62ec61</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KIN_MODE</type>
      <name>m_mode</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a7358520439c495b43a0600412335c6d1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_path_points</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a03b92b089be7dc739477d17becfbdbca</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; KDL::Frame &gt;</type>
      <name>m_frames</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>a391abad2809c9f3c0c39e2df15e4b59e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; double &gt; &gt;</type>
      <name>m_joints</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>abfb942efc5ae20629f7a8d3a4f905608</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; boost::icl::interval_set&lt; double &gt; &gt; &gt;</type>
      <name>m_intervals</name>
      <anchorfile>classKinematicsModule.html</anchorfile>
      <anchor>adaeb327716eb068265e467c7c5043664</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>KinematicSolver</name>
    <filename>classKinematicSolver.html</filename>
    <member kind="function">
      <type></type>
      <name>KinematicSolver</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a6039281012a5a8d96e3b16163c79d22a</anchor>
      <arglist>(const unsigned int nj, const double tol, const std::vector&lt; double &gt; &amp;links, const std::vector&lt; double &gt; &amp;limits, const std::vector&lt; std::vector&lt; double &gt; &gt; &amp;dh)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~KinematicSolver</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a3e38500a085c011c34080970558f3bc5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ForwardKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a2483eaac0ae5927a2c3c823c65f65edc</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;joints, unsigned int &amp;gc, double &amp;psi, KDL::Frame &amp;pose)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>SingleInverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a7cbedb1a566fcb000806511e51249641</anchor>
      <arglist>(const KDL::Frame &amp;pose, const unsigned int gc, const double psi, std::vector&lt; double &gt; &amp;joints)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>IterationInverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a78079360cf3161a6316f8551dd88faa4</anchor>
      <arglist>(const KDL::Frame &amp;pose, const unsigned int gc, const double st_psi, std::vector&lt; double &gt; &amp;joints, double &amp;new_psi)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>BatchInverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>af589fd5fe9e9be29bb4467a4dee8ed36</anchor>
      <arglist>(const std::vector&lt; KDL::Frame &gt; &amp;poses, const unsigned int gc, const double st_psi, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;joints)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>SinglePoseScores</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a0056fa5bf98bd599650fb7146f4e649a</anchor>
      <arglist>(const KDL::Frame &amp;pose, const unsigned int gc, const double psi, std::vector&lt; double &gt; &amp;scores)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>BatchPsiIntervals</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a2513938176314eb590da6eba95c30ca7</anchor>
      <arglist>(std::vector&lt; KDL::Frame &gt; &amp;poses, const unsigned int gc, std::vector&lt; boost::icl::interval_set&lt; double &gt; &gt; &amp;intrvls)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>CheckNullspace</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a1b2102808cac5da4b3875e6345923556</anchor>
      <arglist>(std::vector&lt; KDL::Frame &gt; &amp;target_frames, const double max_nschange, std::vector&lt; unsigned int &gt; &amp;gc_order, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;initial_psi)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ArmAngleScore</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a48a098b06fb90b75c537b99d575edfc0</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;interval, const double current_nsparam, const double max_nschange, std::vector&lt; double &gt; &amp;ns_scores)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>GlobConfOrder</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a12bd33a07505eb88bd96c9ffbf985388</anchor>
      <arglist>(const unsigned int gc, std::vector&lt; unsigned int &gt; &amp;possible, std::vector&lt; unsigned int &gt; &amp;gc_order)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>SingleInverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a65853a7d06b526f590dddb4d885efa7f</anchor>
      <arglist>(const Eigen::Matrix4d &amp;pose, const unsigned int gc, const double psi, std::vector&lt; double &gt; &amp;joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>int</type>
      <name>IterationInverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a463b1c7fd77ff51a18d7c11074b3935f</anchor>
      <arglist>(const Eigen::Matrix4d &amp;pose, const unsigned int gc, const double st_psi, std::vector&lt; double &gt; &amp;joints, double &amp;new_psi)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>int</type>
      <name>BatchInverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a78fd89b6df5de61729cef2613b3d5fe0</anchor>
      <arglist>(const std::vector&lt; Eigen::Matrix4d &gt; &amp;poses, const unsigned int gc, const double st_psi, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>ReferencePlane</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a7f8df6aafc448d4f4d2f887b3bcc38a9</anchor>
      <arglist>(const Eigen::Matrix4d &amp;pose, const int elbow, Eigen::Vector3d &amp;ref_plane, Eigen::Matrix3d &amp;rot_elbow)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>CoeffCalculation</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a071fdf2667a9621345ee349e41639f36</anchor>
      <arglist>(const Eigen::Matrix4d &amp;pose, const unsigned int gc, std::vector&lt; std::vector&lt; double &gt; &gt; &amp;coeffs, double &amp;j4)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>InverseKinematics</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a01eaa5e9cb323dda62f27378449b25c7</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; double &gt; &gt; &amp;coeffs, const double j4, const unsigned int gc, const double psi, std::vector&lt; double &gt; &amp;joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>PsiFunction</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a075db8ee24613e5bc81ae9431324f054</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;coeff, const double theta, std::vector&lt; double &gt; &amp;psi)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>PivotLimits</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>ac6e77252d8a734862603c1863a3f239d</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;coeff, const double limit, std::vector&lt; double &gt; &amp;intervals)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>HingeLimits</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a14dff0ff5a20d7322daef88648dc7925</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;coeff, const int conf, const double limit, std::vector&lt; double &gt; &amp;intervals)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>PsiIntervals</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a60cd10079d49d0d023eb6b2eb4a57109</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; double &gt; &gt; &amp;coeffs, const unsigned int gc, boost::icl::interval_set&lt; double &gt; &amp;feasible)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>CheckSolution</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>afa6091a170e8f073125c12861473d5b9</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;coeff, const double jl, const double psi)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>IterationNewPsi</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a6ac2633dc44a9212e91071f8723ef3cc</anchor>
      <arglist>(const boost::icl::interval_set&lt; double &gt; &amp;feasible, const double cur_psi, double &amp;new_psi)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>BatchNewPsi</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>afdb3ba5d4b04f105633d5bba1809b7c3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>GlobConfOrder</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a46143c4c1e9400d15087da60f3869d4a</anchor>
      <arglist>(const unsigned int gc, std::vector&lt; unsigned int &gt; &amp;gc_order)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>ShadowIntervals</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a4e74bf87abaa60098eae292390866036</anchor>
      <arglist>(const std::vector&lt; boost::icl::interval_set&lt; double &gt; &gt; &amp;intervals, const double max_nschange, std::vector&lt; boost::icl::interval_set&lt; double &gt; &gt; &amp;shadow)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>IntersectIntervals</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>aba7f7fffccb7d56e78d28b1bff5f8f38</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; double &gt; &gt; &amp;intervals, boost::icl::interval_set&lt; double &gt; &amp;interval_out)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>IntersectIntervals</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a5245837c5484b8bc6627fe7b6bbfecc3</anchor>
      <arglist>(const std::vector&lt; boost::icl::interval_set&lt; double &gt; &gt; &amp;intervals, boost::icl::interval_set&lt; double &gt; &amp;interval_out)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>Eigen::Matrix3d</type>
      <name>skew_symmetric</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>afb0f1beba844fa8659274f3266f0f839</anchor>
      <arglist>(const Eigen::Vector3d &amp;vec)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>Eigen::Matrix4d</type>
      <name>denavit_hartenberg</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a803485de3b9a5ddf8a3010b136219749</anchor>
      <arglist>(const double a, const double alpha, const double d, const double theta)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>KDL::Frame</type>
      <name>eigen2kdlframe</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a212101f201be16b83206c7813f1639cb</anchor>
      <arglist>(const Eigen::Matrix4d &amp;mat)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>Eigen::Matrix4d</type>
      <name>kdlframe2eigen</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>afe50e7f06dbde3ec075786db96fc366c</anchor>
      <arglist>(KDL::Frame &amp;T)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>iclinterval2vector</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a38d4929d41353e53c51b5e90458a82c5</anchor>
      <arglist>(boost::icl::interval_set&lt; double &gt; &amp;interval)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a3b98945784dca445ff81210a0a564d85</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_tol</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>ad8fb469a8608e37ef3bfb32a901863ed</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_dist_sing</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>aefb752a19bc70d25e6c3773d66f00495</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_lnk</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>ae1f40c110f9196183308401a10f7eba4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_lim</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a517122e44ef828c1f36369a82348902a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; double &gt; &gt;</type>
      <name>m_dh</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a1c174569365545539af5c174be91853f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>NullspaceGenerator</name>
    <filename>classNullspaceGenerator.html</filename>
    <member kind="function">
      <type></type>
      <name>NullspaceGenerator</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a59d5cc0e77797de6d6c676c4d23fc331</anchor>
      <arglist>(const unsigned int nj, const double tol, std::vector&lt; double &gt; &amp;links)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~NullspaceGenerator</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>ad9a94fb9357a78c73adee8c5dd18f2fe</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>computeValidElbowAngles</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a3858be41d718c45689e5ea84b331be54</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;jointlimits, const KDL::Frame &amp;target, const unsigned int &amp;rconfiguration, double blockedJoints[7][4][2], double blockedJointsCombined[11][2])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>mergeIntervals</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a8b198082cd3ab0b5220eac19cff54f8e</anchor>
      <arglist>(const double src[][2], const int n, double dst[][2])</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>computeValidElbowAnglesPart</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a05a4d3fe32e4e9f17c1700ec16bbd5b5</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;jointlimits, const double &amp;l1, const double &amp;l2, const KDL::Vector &amp;target, double blocked[3][4][2], bool &amp;allblocked, const bool &amp;inv1, const bool &amp;inv3)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>quicksortIntervals</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>afe9a3bb26529ee9f7966e2a07bfcf45c</anchor>
      <arglist>(double arr[][2], int left, int right)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>double</type>
      <name>cotan</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a9a1b2554f058deec1748a57008cc5589</anchor>
      <arglist>(double x)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>double</type>
      <name>anglemod</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>ab2d3753e776994435b721658fd936d59</anchor>
      <arglist>(double x)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a4f31378b52302e3c24530bc495e9eb59</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_tol</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a25b21729477d44c5ae1a0c72f0b40fab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_links</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>acbf91a7ba1914929871bb36e73ab2543</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>tolerance_prop</name>
      <anchorfile>classNullspaceGenerator.html</anchorfile>
      <anchor>a12659111293504b2605fbe76c51904d7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>NullspaceSolver</name>
    <filename>classNullspaceSolver.html</filename>
    <member kind="function">
      <type></type>
      <name>NullspaceSolver</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>af15139a93c86f4c9e3d2d82a354855d4</anchor>
      <arglist>(const double tolerance=0.0001)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~NullspaceSolver</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>adfc7383652f4bf1d3c78d1c14d6f1229</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_current_rconf</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a3357ab4b117c50c86cf2d4f4efd6f2c0</anchor>
      <arglist>(unsigned int rconf)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_current_nsparam</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a906e5943e8c47df5e178416c43bada40</anchor>
      <arglist>(double nsparam)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_trajectory_nsparam</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a5144459ce12e6bd1cc35ca19fdccecb5</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;nsparam)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_order_rconf</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>adde6d6bcfbb4e80a2eb7d7f76e29b420</anchor>
      <arglist>(std::vector&lt; unsigned int &gt; &amp;ord_rc)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_ns_score</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a22f50407d2e4171cfc578cb1929befc9</anchor>
      <arglist>(std::vector&lt; std::vector&lt; double &gt; &gt; &amp;ns_score)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>AddTrajectoryNS</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>af302a90144b38ac129079f062b510132</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; std::vector&lt; std::pair&lt; double, double &gt; &gt; &gt; &gt; &amp;rconf_intervals, const unsigned int rconf, const double nsparam, const double max_nschange, const double ns_var)</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>CalculatePossibleInitialNS</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a39bb8a8d0493d1cadaad3a40b76704e0</anchor>
      <arglist>(const std::vector&lt; bool &gt; &amp;flags)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>CalculateTrajectoryNS</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a4e02dec81ccd5e7acbe782fd07f58d73</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>CalculateNSScore</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>aef5bc91ff3bbe60b78542f68329c1772</anchor>
      <arglist>(const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;interval, const double current_nsparam, const double max_nschange, std::vector&lt; double &gt; &amp;ns_scores)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>NegativeNS</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a421156c842843f4b37badaec3493ee48</anchor>
      <arglist>(const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;input, std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;output) const </arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>MergeIntervals</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a552cbc66262b8b239d47e5eeef62cdc9</anchor>
      <arglist>(const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intervals1, const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intervals2, std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;merged_interval) const </arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>IntersectIntervals</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a3dd64aacc9960687cfaf39fd5a5354ed</anchor>
      <arglist>(const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intervals1, const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intervals2, std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;inters_interval) const </arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>IntersectIntervals</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a48561f19506e2d3e256cf6d861564eda</anchor>
      <arglist>(const std::pair&lt; double, double &gt; &amp;pairs, const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intervals, std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intersection) const </arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>DistToLimits</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a6472f168017f2625898ebf37a3cfbef5</anchor>
      <arglist>(const std::vector&lt; std::pair&lt; double, double &gt; &gt; &amp;intervals, const double value, std::pair&lt; double, double &gt; &amp;dist_limits) const </arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>CheckStartingNS</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>ad1ea18594e97f2095988a081354e8573</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; std::pair&lt; double, double &gt; &gt; &gt; &amp;input, const double value, const double ns_lim, const double max_nschange)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>FilterNSIntervals</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a4c3b83a9108a1a1aa412a93f9eb76671</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; std::pair&lt; double, double &gt; &gt; &gt; &amp;input, const double value, std::vector&lt; std::vector&lt; std::pair&lt; double, double &gt; &gt; &gt; &amp;output)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>CalculateRConfOrder</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a764b1a33648ae2692e5a8adcc806c9a5</anchor>
      <arglist>(const unsigned int rconf, std::vector&lt; unsigned int &gt; &amp;rconf_order)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>CreateShadowNS</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a3c990da4270909800b6ea049ba51cd53</anchor>
      <arglist>(const unsigned int rconf)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_tolerance</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>ad33c3e3a9357e7ee0cec538a72df48d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_current_initial_rconf</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a61670c6f0b799206b550c4a0cd0c899c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_target_initial_rconf</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a55c4f63f8017e5a647b953f0a84d6085</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_current_initial_nsparam</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>aeb703434dcd84464b9c65c19337c8470</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_target_initial_nsparam</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>ab20699c9cc8237d741582bc14adf9b59</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_nsparam_variation</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>ae228ce669079c5ef2deaf7496c0c36dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_max_nschange</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>aed9ae112657861f2be9a99b4ac0d2eae</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_exp_b1</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a989d7f6afd77b6448e55fc877f4c8807</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_exp_b2</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a5a48b2627b09efc9230356672083bc7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; unsigned int &gt;</type>
      <name>m_order_rconf</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a76e00e8843e5360aeb5cd6f9a1e0b830</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; double &gt; &gt;</type>
      <name>m_ns_score</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a5a9414012c9a07653f6864dd2c30da6d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; std::vector&lt; std::pair&lt; double, double &gt; &gt; &gt; &gt;</type>
      <name>m_intervals</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>a9d6bf4d51d3c3c92d287e14d58d9fd53</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; std::pair&lt; double, double &gt; &gt; &gt;</type>
      <name>m_shadow</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>ad0238c319d1f74d163c0082db1a58123</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_nsparam</name>
      <anchorfile>classNullspaceSolver.html</anchorfile>
      <anchor>ae05e49afbe274a702ce48bf181567ca0</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2</name>
    <path>/home/carlos/catkin_ws2/</path>
    <filename>dir_59fc39b2c45567e1121c4509a7cb21ab.html</filename>
    <dir>/home/carlos/catkin_ws2/src</dir>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/kinematics_module/kinematics</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/kinematics/</path>
    <filename>dir_ca58a7e74964a447d52e205bd295d991.html</filename>
    <file>global_configuration.h</file>
    <file>kinematics.cpp</file>
    <file>kinematics.h</file>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/kinematics_module</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/</path>
    <filename>dir_7804df0e408dd89b3e4e00a938a35b22.html</filename>
    <dir>/home/carlos/catkin_ws2/src/kinematics_module/kinematics</dir>
    <dir>/home/carlos/catkin_ws2/src/kinematics_module/nullspace</dir>
    <dir>/home/carlos/catkin_ws2/src/kinematics_module/src</dir>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/kinematics_module/nullspace</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/nullspace/</path>
    <filename>dir_1fc3815a35fc1fe146400a175ae81753.html</filename>
    <file>nullspace_generator.cpp</file>
    <file>nullspace_generator.hpp</file>
    <file>nullspace_solver.cpp</file>
    <file>nullspace_solver.hpp</file>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/kinematics_module/src</name>
    <path>/home/carlos/catkin_ws2/src/kinematics_module/src/</path>
    <filename>dir_4c8b5df5343e82d21d888677f8ae31c9.html</filename>
    <file>kinematics_module-component.cpp</file>
    <file>kinematics_module-component.hpp</file>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src</name>
    <path>/home/carlos/catkin_ws2/src/</path>
    <filename>dir_7f2f2864f55f29a6508ca72d40d11a85.html</filename>
    <dir>/home/carlos/catkin_ws2/src/kinematics_module</dir>
  </compound>
</tagfile>
