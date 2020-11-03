<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>kinematics.cpp</name>
    <path>/home/carlos/catkin_ws2/src/robot_state/kinematics/</path>
    <filename>kinematics_8cpp</filename>
    <includes id="kinematics_8hpp" name="kinematics.hpp" local="yes" imported="no">kinematics.hpp</includes>
    <member kind="define">
      <type>#define</type>
      <name>_USE_MATH_DEFINES</name>
      <anchorfile>kinematics_8cpp.html</anchorfile>
      <anchor>a525335710b53cb064ca56b936120431e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>kinematics.hpp</name>
    <path>/home/carlos/catkin_ws2/src/robot_state/kinematics/</path>
    <filename>kinematics_8hpp</filename>
    <class kind="class">KinematicSolver</class>
  </compound>
  <compound kind="file">
    <name>robot_state-component.cpp</name>
    <path>/home/carlos/catkin_ws2/src/robot_state/src/</path>
    <filename>robot__state-component_8cpp</filename>
    <includes id="robot__state-component_8hpp" name="robot_state-component.hpp" local="yes" imported="no">robot_state-component.hpp</includes>
    <member kind="function" static="yes">
      <type>static const string</type>
      <name>PROP_FILE</name>
      <anchorfile>robot__state-component_8cpp.html</anchorfile>
      <anchor>a01ac56511b891cfbc2d07e6b78bce6ec</anchor>
      <arglist>(&quot;/home/carlos/catkin_ws2/src/Properties/KUKA_LBR_IIWA.xml&quot;)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>robot_state-component.hpp</name>
    <path>/home/carlos/catkin_ws2/src/robot_state/src/</path>
    <filename>robot__state-component_8hpp</filename>
    <includes id="kinematics_8hpp" name="kinematics.hpp" local="yes" imported="no">kinematics.hpp</includes>
    <class kind="class">RobotState</class>
  </compound>
  <compound kind="class">
    <name>KinematicSolver</name>
    <filename>classKinematicSolver.html</filename>
    <member kind="function">
      <type></type>
      <name>KinematicSolver</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a6de915f9546b79dcc8670cea8cb9f441</anchor>
      <arglist>(const unsigned int nj, const double tol, std::vector&lt; double &gt; &amp;links, std::vector&lt; double &gt; &amp;x_min, std::vector&lt; double &gt; &amp;x_max, std::vector&lt; std::vector&lt; double &gt; &gt; dh)</arglist>
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
      <name>calculateFK</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a7c410e11ad3e3286607efbcdd68f842a</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;jointValues, unsigned int &amp;rconfiguration, double &amp;nsparam, KDL::Frame &amp;targetmatrix, KDL::Vector &amp;elbow_tangent)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>calculateIK</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a0eb6bb3a429860b4cbc95afdb0aefb77</anchor>
      <arglist>(const KDL::Frame &amp;targetmatrix, const unsigned int rconfiguration, const double nsparam, std::vector&lt; double &gt; &amp;jointValues)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>calculateIK</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a6234e77219bdb7bd11bca806c6174317</anchor>
      <arglist>(const KDL::Frame &amp;targetmatrix, const double nsparam, double rconf_jointValues[8][7])</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a4f072a020bf5aee1d4293ad001de7cb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_tol</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>ac2e7717cf49376874198a247e1390e51</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_links</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>ad96b63deab75ab05585944c34caade3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_xmin</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>ab1f2669d8e97865162864eeeb66be33d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_xmax</name>
      <anchorfile>classKinematicSolver.html</anchorfile>
      <anchor>a4b50eea32d24247f7c7f2e4e58a7953d</anchor>
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
    <name>RobotState</name>
    <filename>classRobotState.html</filename>
    <member kind="function">
      <type></type>
      <name>RobotState</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a86931bd7a4433cf6f242e6492dd2da8f</anchor>
      <arglist>(const std::string &amp;component_name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>configureHook</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>ac709fc1ed4af3b06b5c8335e63847850</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>startHook</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>aaca0bf6537bb172117aadf2c9e5744de</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateHook</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a840fb9b91537184476651d6c5e51e7e9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopHook</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a22c6e33a865ec9ee2b709016e0b53251</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cleanupHook</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>aaf4d001d412560203c7b63feff4eab81</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a007d02114e1a59973a5586b38ad73ba5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>double</type>
      <name>m_tolerance</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a4994b8e31e4d059a3762f8cb620aa97b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_xmin</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>ab3f898878994498294a50d93d12fec51</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_xmax</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>aa77407941ea85242a928657a4f8b77fd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_limbs</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a4fd0fe0f01b5d299f4664e90d0959dd0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_a</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a8136f5d5f17bc2be7d29b99ebfa80e6b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_alpha</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a58ec2c00909b59fefb76281cc8f78038</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_d</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>ab0b0eb3505e34676e9f3625f19df5493</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_dh_theta</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a385325b5895f4fb612a5b12072dd4efd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; std::vector&lt; double &gt; &gt;</type>
      <name>m_dh</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a61e658c2a11c4acb7d99a2b9031e1615</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>inport_current_joint_positions</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a6bbda96fdf840ac4d0e61431ff559c66</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>inport_current_joint_velocities</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a8ed6cff511a72a3cabb8f2ee0514643e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; robot_msgs::Posture &gt;</type>
      <name>outport_robot_param</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a0727ad7c7a227613c5616e968102f290</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>outport_current_joint_positions</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a83abee321a5d9dfaf6451a7e3dd8eb2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>outport_current_joint_velocities</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a94e4a26da2adf8a163f2695f7178b648</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>enablePublishers</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a6405dc5a6f173c6f4b4ae305f5d4c26e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>disablePublishers</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a18e514c3371d70c0ef57badeaec8f0b1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>drawPlot</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a628495c299198046e678b51806e23c5b</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;attribute)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>updatePlots</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>aa56081cdc65bd185aa0acfffb6e2fcd6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>Plotter</type>
      <name>m_plotter</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a93ac5eb16097a88fbaaca9c4b1e2d272</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>boost::chrono::system_clock</type>
      <name>m_clock</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a1ffcf0b7c8223f62651f806dd50c252a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>boost::chrono::system_clock::time_point</type>
      <name>m_start_time</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a6e7aba090649825bcfa3b79167635e1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; KinematicSolver &gt;</type>
      <name>m_kinematic_solver</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>aeda991d1d470d4e651507bb016368776</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_cx</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a7b56ee6a95a993ae2997609742940f6d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_cv</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>ad90a5b9e3e65b301a5599082e4488311</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_frame_pos</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>ad02f29ee4557804692d566da7b0e3bf9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_frame_rot</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a413fdf206d27f4b2c1d1d7817fd31b16</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_nsparam</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>af2145e06c4b3410349aff103dbd125e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_rconf</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a47bee04ffb758ff2b8f3d5297d59e2ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Vector</type>
      <name>m_elbowtg</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a911067957cad790905bab6e9835216f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_frame</name>
      <anchorfile>classRobotState.html</anchorfile>
      <anchor>a1d4bf3dd962a4d2e3dfa31cf7f1ffbd5</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/robot_state/kinematics</name>
    <path>/home/carlos/catkin_ws2/src/robot_state/kinematics/</path>
    <filename>dir_c3fd868e7ef329c465f1721fe663cb13.html</filename>
    <file>kinematics.cpp</file>
    <file>kinematics.hpp</file>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/robot_state/src</name>
    <path>/home/carlos/catkin_ws2/src/robot_state/src/</path>
    <filename>dir_68267d1309a1af8e8297ef4c3efbcdba.html</filename>
    <file>robot_state-component.cpp</file>
    <file>robot_state-component.hpp</file>
  </compound>
</tagfile>
