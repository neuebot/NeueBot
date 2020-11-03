<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>motion_profile_polynomials.cpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/motion_through_points/</path>
    <filename>motion__profile__polynomials_8cpp</filename>
    <includes id="motion__profile__polynomials_8hpp" name="motion_profile_polynomials.hpp" local="yes" imported="no">motion_profile_polynomials.hpp</includes>
  </compound>
  <compound kind="file">
    <name>motion_profile_polynomials.hpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/motion_through_points/</path>
    <filename>motion__profile__polynomials_8hpp</filename>
    <class kind="class">MotionProfilePolynomials</class>
  </compound>
  <compound kind="file">
    <name>motion_through_points_trajectory_generator.cpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/motion_through_points/</path>
    <filename>motion__through__points__trajectory__generator_8cpp</filename>
    <includes id="motion__through__points__trajectory__generator_8hpp" name="motion_through_points_trajectory_generator.hpp" local="yes" imported="no">motion_through_points_trajectory_generator.hpp</includes>
  </compound>
  <compound kind="file">
    <name>motion_through_points_trajectory_generator.hpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/motion_through_points/</path>
    <filename>motion__through__points__trajectory__generator_8hpp</filename>
    <includes id="motion__profile__polynomials_8hpp" name="motion_profile_polynomials.hpp" local="yes" imported="no">motion_profile_polynomials.hpp</includes>
    <class kind="class">MotionThroughPointsTrajectoryGenerator</class>
  </compound>
  <compound kind="file">
    <name>point_to_point_trajectory_generator.cpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/point_to_point/</path>
    <filename>point__to__point__trajectory__generator_8cpp</filename>
    <includes id="point__to__point__trajectory__generator_8hpp" name="point_to_point_trajectory_generator.hpp" local="yes" imported="no">point_to_point_trajectory_generator.hpp</includes>
  </compound>
  <compound kind="file">
    <name>point_to_point_trajectory_generator.hpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/point_to_point/</path>
    <filename>point__to__point__trajectory__generator_8hpp</filename>
    <class kind="class">PointToPointTrajectoryGenerator</class>
  </compound>
  <compound kind="file">
    <name>joint_trajectory_controller-component.cpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/src/</path>
    <filename>joint__trajectory__controller-component_8cpp</filename>
    <includes id="joint__trajectory__controller-component_8hpp" name="joint_trajectory_controller-component.hpp" local="yes" imported="no">joint_trajectory_controller-component.hpp</includes>
    <member kind="function" static="yes">
      <type>static const string</type>
      <name>PROP_FILE</name>
      <anchorfile>joint__trajectory__controller-component_8cpp.html</anchorfile>
      <anchor>a01ac56511b891cfbc2d07e6b78bce6ec</anchor>
      <arglist>(&quot;/home/carlos/catkin_ws2/src/Properties/KUKA_LBR_IIWA.xml&quot;)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>joint_trajectory_controller-component.hpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/src/</path>
    <filename>joint__trajectory__controller-component_8hpp</filename>
    <includes id="joint__trajectory__controller-exceptions_8hpp" name="joint_trajectory_controller-exceptions.hpp" local="yes" imported="no">joint_trajectory_controller-exceptions.hpp</includes>
    <includes id="point__to__point__trajectory__generator_8hpp" name="point_to_point_trajectory_generator.hpp" local="yes" imported="no">point_to_point_trajectory_generator.hpp</includes>
    <includes id="motion__through__points__trajectory__generator_8hpp" name="motion_through_points_trajectory_generator.hpp" local="yes" imported="no">motion_through_points_trajectory_generator.hpp</includes>
    <class kind="class">JointTrajectoryController</class>
  </compound>
  <compound kind="file">
    <name>joint_trajectory_controller-exceptions.hpp</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/src/</path>
    <filename>joint__trajectory__controller-exceptions_8hpp</filename>
    <class kind="class">JTCException</class>
  </compound>
  <compound kind="class">
    <name>JointTrajectoryController</name>
    <filename>classJointTrajectoryController.html</filename>
    <member kind="function">
      <type></type>
      <name>JointTrajectoryController</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ac7421a38bfe80f70fa257b9bf9381d8e</anchor>
      <arglist>(const std::string &amp;component_name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>configureHook</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a65f7026cca1cec67f178e1e06a500282</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>startHook</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a6d2d41ffc4127fb491c12dc9a5b266b5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateHook</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a3e1bef227ffa9200cc23481e930afd5d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopHook</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>af2d7adc369333081b1289b18259e3697</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cleanupHook</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ace18eaf9f9cd4e41fb531a7e1408fea9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>errorHook</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a08c294f3edb7b6d59659aa5927a842c9</anchor>
      <arglist>(int ResultValue)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a7f031d5f1de86b852febf6b6ff03a08e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>double</type>
      <name>m_maxv</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>aeb853f31ee761c891d26868b96f6da38</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>double</type>
      <name>m_maxa</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>afaf35fa149e4149d47d7260d08e15617</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>double</type>
      <name>m_tolerance</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>abc202b1380d20f6fe2973916c85e5275</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>inport_current_joint_positions</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>aca3dda3eb0e2bd0ab2e06ff2e2b9832a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>inport_current_joint_velocities</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ac41ac0ab1c61a164992a2de1652bdec2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; std::vector&lt; double &gt; &gt;</type>
      <name>outport_next_joint_positions</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a4243913f07f5224d3494d17bb2746e4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; int &gt;</type>
      <name>outport_error_code</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ad240cf6d0482e02a878218ce313d6fe3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration" protection="private">
      <type></type>
      <name>MOTION_TYPE</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ad57796d0c069d8214b759fa0df378e5e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>PTP</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ad57796d0c069d8214b759fa0df378e5eaf0a7558b8ec5c1aa5b0ffea686d25a80</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>MTP</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ad57796d0c069d8214b759fa0df378e5eaeea1b7fd02119622733da63fffa4040f</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>DrawPlot</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a080de31425efe9cfae74b6dec8f8530b</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;attribute)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>UpdatePlots</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>aede4806af65e9215d23c6043cbed650f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>StopArm</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a9e879bec0f3a08c8d0c5ff9b8331612d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>SetMotionThroughPoints</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ab4b72a1b41b269ccb7bad6933994cd17</anchor>
      <arglist>(const std::vector&lt; std::vector&lt; double &gt; &gt; &amp;target_joints, const std::vector&lt; double &gt; &amp;m_target_times)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>SetPointToPoint</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a4f205d8dcd4bf63852e6239a1282ecbc</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;target_joints)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>RecalculateTimes</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a4113b871389c687a7baff0d889298cf9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>IsRobotStopped</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ac0e2f9ca264ea07744b24d2f8fd90d60</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_dt</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a4ef5904ac5302451101a6812b0a74998</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>m_moving</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ab1c7e9d6382a7f361b2b010e74c8d140</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>MOTION_TYPE</type>
      <name>m_mt</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a3b888d473d04be941ea448f2f2525040</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_q_curr</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ad91bd9bd49678da9f746665b41444673</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_v_curr</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>af119c8332b059baa8f9e6910c1df4533</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_q_tar_serial</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a8d1d07ffd48c9f0844b8376bd8285b3e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::vector&lt; double &gt; &gt;</type>
      <name>m_q_tar</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ada08bb50183892493234ad3d7020bb52</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_t_tar</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a5c80973750ca7356a4fa89104c936630</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; double &gt;</type>
      <name>m_q_next</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>abf3ca213f50807e92bd52a219ae348d9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; RTT::os::Mutex &gt;</type>
      <name>m_mutex</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>aeb289341fa19c3365a60fa30adced4d7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; PointToPointTrajectoryGenerator &gt;</type>
      <name>PointToPoint</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a10a9f72bac54d724f3bdb6571e5d2530</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; MotionThroughPointsTrajectoryGenerator &gt;</type>
      <name>MotionThroughPoints</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ab6d5469657d297cddc053f67652858d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>Plotter</type>
      <name>m_plotter</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ad2ec5cdc2bd820f44b3e53ac1e64eaed</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>boost::chrono::system_clock</type>
      <name>m_clock</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>a5fe7754d35cdec82ef2050dd0f6f96fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>boost::chrono::system_clock::time_point</type>
      <name>m_start_time</name>
      <anchorfile>classJointTrajectoryController.html</anchorfile>
      <anchor>ab9a3f841cef378d877d8c9b2abf64bed</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>JTCException</name>
    <filename>classJTCException.html</filename>
    <member kind="function">
      <type></type>
      <name>JTCException</name>
      <anchorfile>classJTCException.html</anchorfile>
      <anchor>a4288887ae9820d2219300dec37f1f03d</anchor>
      <arglist>(const string &amp;message)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~JTCException</name>
      <anchorfile>classJTCException.html</anchorfile>
      <anchor>ab8969a058a3fae85243d86123969d38c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>what</name>
      <anchorfile>classJTCException.html</anchorfile>
      <anchor>a45277c829c873c39ca5f0c901c2f6b80</anchor>
      <arglist>() const </arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>string</type>
      <name>msg_</name>
      <anchorfile>classJTCException.html</anchorfile>
      <anchor>a2bc2e5064bb04eaa2320bb99926e1600</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>MotionProfilePolynomials</name>
    <filename>classMotionProfilePolynomials.html</filename>
    <member kind="function">
      <type></type>
      <name>MotionProfilePolynomials</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a658389acebdbb173864261a9e5f0dd5b</anchor>
      <arglist>(const vector&lt; double &gt; &amp;via_points, const vector&lt; double &gt; &amp;time_via_points, const double &amp;initial_velocity, const double &amp;end_velocity, const double &amp;initial_acceleration, const double &amp;end_acceleration)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ComputePolynomials</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>aabb6ebfd2f0e3bbc4d68f818d6162063</anchor>
      <arglist>(vector&lt; vector&lt; double &gt; &gt; &amp;poly)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>N</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a2e0afcd92aa49ad9f0a4d9c7c3959409</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>m_q</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a5ea005b4c79f7349bdd68b57d5cb5468</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>m_v</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a99f99cd1feb888b52c4051218fdf4a41</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>m_a</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>aa21536a0b76f87ec94235c22eae5e866</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>m_t</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>ab777d39593eca3fa10b579bb9abffe7f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>m_dt</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>ad8650956af2c1a5a5bfc8a49d5f89c8d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_vi</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a3d8e12f3785f89991f3f189763ced267</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_vf</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>af66df727bf0c7cd9e0a5913c6e859e48</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_ai</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a12708e9681ff3eb86cc10fa6f9decf3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_af</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a13c3673398bebab9017ff177b1d80c5b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; vector&lt; double &gt; &gt;</type>
      <name>m_poly</name>
      <anchorfile>classMotionProfilePolynomials.html</anchorfile>
      <anchor>a771619d384e013643f63fd3acc5f3ba4</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>MotionThroughPointsTrajectoryGenerator</name>
    <filename>classMotionThroughPointsTrajectoryGenerator.html</filename>
    <member kind="function">
      <type></type>
      <name>MotionThroughPointsTrajectoryGenerator</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a5a725331dfa59101779d697e47c4cb8c</anchor>
      <arglist>(const unsigned int &amp;n_joints, const double &amp;cycle_time, const double &amp;max_vel, const double &amp;max_acc)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>InitiateNewMotion</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>ad490b9237a4ac72e914c6df740245c7c</anchor>
      <arglist>(const vector&lt; double &gt; &amp;current_joint_positions, const vector&lt; double &gt; &amp;current_joint_velocities, const vector&lt; vector&lt; double &gt; &gt; &amp;via_points, const vector&lt; double &gt; &amp;time_via_points, const vector&lt; double &gt; &amp;init_vel, const vector&lt; double &gt; &amp;end_vel, const vector&lt; double &gt; &amp;init_acc, const vector&lt; double &gt; &amp;end_acc)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>GenerateNextMotionState</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>adb412351025bdedd9bf46f9403bab9c9</anchor>
      <arglist>(vector&lt; double &gt; &amp;next_joint_positions)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_dt</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>ab431799ac6dda55b9cbd8e4c7da29e5b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_v_max</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>ad70c3d7b2822e4dfb8fa4d23062ef7b3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_a_max</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a83d54ec061bea060685877a4dffd173d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>af4cbbc43df65f97febaef28eed83515f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>x_min</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a11ba93223585e0cd392a808bffa6668a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>x_max</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>ac0e9df40d66a1fde74529fd234efa164</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>v_max</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>ac2418aa42dbb44b0413b4f3cc9c711dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>a_max</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a151da15d00ea099f6e17ba3130d70d01</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_num_it</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a7bb55ce851e20034378273aa93ff2366</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; vector&lt; double &gt; &gt;</type>
      <name>m_q_it</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a26d955de4560e62f4dceec7c7bebaad9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; vector&lt; double &gt; &gt;</type>
      <name>m_v_it</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a5036b091cc96a1d2a4da930d233e6d27</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; vector&lt; double &gt; &gt;</type>
      <name>m_a_it</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a53c569c9976ef654709950abd4cea338</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>vector&lt; double &gt;</type>
      <name>m_t_it</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>ae07e04bd9e341fbb2cc91152a0602c23</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>m_index_it</name>
      <anchorfile>classMotionThroughPointsTrajectoryGenerator.html</anchorfile>
      <anchor>a27ba4a58a85dd1b40734a0d48d0b044f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>PointToPointTrajectoryGenerator</name>
    <filename>classPointToPointTrajectoryGenerator.html</filename>
    <member kind="function">
      <type></type>
      <name>PointToPointTrajectoryGenerator</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>aafa561992bcc239797c1cce07102a345</anchor>
      <arglist>(const unsigned int &amp;n_joints, const double &amp;cycle_time, const double &amp;max_vel, const double &amp;max_acc)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~PointToPointTrajectoryGenerator</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>aa4c7f9d34d2d57494e8c5c8807112555</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>InitiateNewMotion</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>a787abfe9ea8e34e875a963d2cfc400db</anchor>
      <arglist>(const vector&lt; double &gt; &amp;current_joint_positions, const vector&lt; double &gt; &amp;current_joint_velocities, const vector&lt; double &gt; &amp;target_joint_positions)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>GenerateNextMotionState</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>a523f3918e407b94a2502bb5732351d74</anchor>
      <arglist>(vector&lt; double &gt; &amp;next_joint_positions, const vector&lt; double &gt; &amp;current_joint_positions)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_v_max</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>a34b2bba2c3d55258bb3e2588fa9e57f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const double</type>
      <name>m_a_max</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>a4d82101123d9d631db6a18ae069add06</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const unsigned int</type>
      <name>m_nj</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>ab92f0ff09ae1254a74f79add798e05ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>m_dt</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>ad7eb35c49b9f7bbfca0ce49cd0262c79</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; ReflexxesAPI &gt;</type>
      <name>RML</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>aca7e05ca92ed3408cd1c34c94d849b95</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; RMLPositionInputParameters &gt;</type>
      <name>IP</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>a3076877d13145518784d87bdffde1954</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; RMLPositionOutputParameters &gt;</type>
      <name>OP</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>af3d54eb1f12ee6a75481776fe1cba3f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>RMLPositionFlags</type>
      <name>Flags</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>aa28b76a4f2f4b2aaa38c684539ff8185</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>int</type>
      <name>ResultValue</name>
      <anchorfile>classPointToPointTrajectoryGenerator.html</anchorfile>
      <anchor>a4ec7e3b7d3c942f088f3cdc9d641954d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/joint_trajectory_controller/motion_through_points</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/motion_through_points/</path>
    <filename>dir_7aea70ca22365ed3d12c87bb5381c154.html</filename>
    <file>motion_profile_polynomials.cpp</file>
    <file>motion_profile_polynomials.hpp</file>
    <file>motion_through_points_trajectory_generator.cpp</file>
    <file>motion_through_points_trajectory_generator.hpp</file>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/joint_trajectory_controller/point_to_point</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/point_to_point/</path>
    <filename>dir_f536a3e5c169612b65931ac744eb2558.html</filename>
    <file>point_to_point_trajectory_generator.cpp</file>
    <file>point_to_point_trajectory_generator.hpp</file>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/joint_trajectory_controller/src</name>
    <path>/home/carlos/catkin_ws2/src/joint_trajectory_controller/src/</path>
    <filename>dir_68267d1309a1af8e8297ef4c3efbcdba.html</filename>
    <file>joint_trajectory_controller-component.cpp</file>
    <file>joint_trajectory_controller-component.hpp</file>
    <file>joint_trajectory_controller-exceptions.hpp</file>
  </compound>
</tagfile>
