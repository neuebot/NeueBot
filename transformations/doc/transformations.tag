<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>transformations-component.cpp</name>
    <path>/home/carlos/catkin_ws2/src/transformations/src/</path>
    <filename>transformations-component_8cpp</filename>
    <includes id="transformations-component_8hpp" name="transformations-component.hpp" local="yes" imported="no">transformations-component.hpp</includes>
    <member kind="define">
      <type>#define</type>
      <name>TODEG</name>
      <anchorfile>transformations-component_8cpp.html</anchorfile>
      <anchor>a6bf807f13685faaeec3dd401195c77de</anchor>
      <arglist>(val)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TORAD</name>
      <anchorfile>transformations-component_8cpp.html</anchorfile>
      <anchor>a216f142ce813db8a78824944cfb50696</anchor>
      <arglist>(val)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static const string</type>
      <name>PROP_FILE</name>
      <anchorfile>transformations-component_8cpp.html</anchorfile>
      <anchor>a01ac56511b891cfbc2d07e6b78bce6ec</anchor>
      <arglist>(&quot;/home/carlos/catkin_ws2/src/Properties/KUKA_LBR_IIWA.xml&quot;)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static const string</type>
      <name>TOOL_FILE</name>
      <anchorfile>transformations-component_8cpp.html</anchorfile>
      <anchor>a1b583b319780542465610a7d109abcb1</anchor>
      <arglist>(&quot;/home/carlos/catkin_ws2/src/Properties/KUKA_TOOLS.xml&quot;)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>transformations-component.hpp</name>
    <path>/home/carlos/catkin_ws2/src/transformations/src/</path>
    <filename>transformations-component_8hpp</filename>
    <class kind="class">Transformations</class>
  </compound>
  <compound kind="class">
    <name>Transformations</name>
    <filename>classTransformations.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>REFERENCE_FRAME</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a48d6a2cea4acf7736161109399d8a2e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>BASE</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a48d6a2cea4acf7736161109399d8a2e0a16cf597015786f81f1cd9a97366e1815</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>END_EFFECTOR</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a48d6a2cea4acf7736161109399d8a2e0a6268406b1e215ee0ca17e17d8001754f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>WORK</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a48d6a2cea4acf7736161109399d8a2e0a23484e8581cf1764d2a660fb823c6eb2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>END_EFFECTORS</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a0c819792e45cd04b1c15c0a4f1a6b548</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>NONE</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a0c819792e45cd04b1c15c0a4f1a6b548a07d0495d371aec3ecb09ac96d3cfa7d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>CAMERA</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a0c819792e45cd04b1c15c0a4f1a6b548ad6f3912c3e03c1d395a1f03426dc2bed</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>EMPTY</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a0c819792e45cd04b1c15c0a4f1a6b548a95fb5cef58535ccec28440f9bc374212</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>PROBE</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a0c819792e45cd04b1c15c0a4f1a6b548ac4059caa039f29949106f263bca9aef7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <type>@</type>
      <name>TREPAN</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a0c819792e45cd04b1c15c0a4f1a6b548ad4d90c1d974a60b7cb0aae77434f2ff1</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Transformations</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>ae96a0a95688608d78109fbed2b596cca</anchor>
      <arglist>(const std::string &amp;component_name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>configureHook</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>ac8dc6897d33d46688604fb5c4689389f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>startHook</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>ac25da2b86ee4bc4a803a39385d7c8e4b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateHook</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a10daa93ca44a90ff2bbde1a383fddfb7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopHook</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>adf1547d4cd1c07eb15454cc41aeb62ab</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cleanupHook</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>aed50b1b6f24780499e27affdca06fb6b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_ee_none_v</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a68d6897dda98254a94f11c99250779f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_ee_empty_v</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a62cdf99668668f2160fecf623876c845</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_ee_probe_v</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>aa46f408387edaa724241272624310807</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_ee_trepan_v</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>addacc56e36291227be7f0184d64dbd1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::InputPort&lt; robot_msgs::Posture &gt;</type>
      <name>inport_robot_posture</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>acdcad5e6deab6e6b5e651c3387ebb9f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RTT::OutputPort&lt; int &gt;</type>
      <name>outport_error_code</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>abb0e07e9a511b68a2b1f02c639b04dae</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>SetBaseTransf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a12b10bb4992dc3369413bb9731927907</anchor>
      <arglist>(const KDL::Frame &amp;base_frame)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>GetBaseTransf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>adb283dcadbb7e1b51f5c8ae9512ef3d4</anchor>
      <arglist>(KDL::Frame &amp;base_frame)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>SetWorkTransf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>ab2ba9f7457415113f24aff39ad3a9225</anchor>
      <arglist>(const KDL::Frame &amp;base_work)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>GetWorkTransf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a39769333a35595b09205138e2b849872</anchor>
      <arglist>(KDL::Frame &amp;base_work)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>SetToolTransf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a3ca01464f4aeb2f24ad05dc6dc64ef2f</anchor>
      <arglist>(const int type)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>GetToolTransf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>ad1ff461ffede679bcea3518c60933857</anchor>
      <arglist>(KDL::Frame &amp;flange_tool)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>GetReferenceFrame</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a4fc482beaf3f60a36ee809091ebea6d2</anchor>
      <arglist>(int reference_id, KDL::Frame &amp;frame)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_ee_none</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a3d05944616df76e52ccc720e18c5b123</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_ee_empty</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a9287fbb3ce496adae7e28238acca3036</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_ee_probe</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a9e29095254dac1786f0e85d644a74f28</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_ee_trepan</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a38bf604f2d139347265b9626dd4417e4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_work_transf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a06d104256b3f0a621c52bad0c7d46900</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_tool_transf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a2945f3f6c2c8839da45d1068b1b83390</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_base_transf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a951592fcc6c148ee3721edc95227aac6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>KDL::Frame</type>
      <name>m_flange_transf</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>a1dd488c0f242d03bc0841471bf88f6f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>m_calibrated</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>aacbf0ff03f18a9149c293fef7adcdec1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>m_online</name>
      <anchorfile>classTransformations.html</anchorfile>
      <anchor>afd12c80b19412e908d00c980cf9ac005</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="dir">
    <name>/home/carlos/catkin_ws2/src/transformations/src</name>
    <path>/home/carlos/catkin_ws2/src/transformations/src/</path>
    <filename>dir_68267d1309a1af8e8297ef4c3efbcdba.html</filename>
    <file>transformations-component.cpp</file>
    <file>transformations-component.hpp</file>
  </compound>
</tagfile>
