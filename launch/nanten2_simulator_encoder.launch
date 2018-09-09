<launch>

  <node pkg = "necst_ros3"
      name = "encoder_az"
      type = "encoder_az.py"
    <param name="name_topic_from" value="encoder_az_input_sim" />
  </node>

  <node pkg = "necst_ros3"
      name = "encoder_el"
      type = "encoder_el.py"
    <param name="name_topic_from" value="encoder_el_input_sim" />
  </node>

  <node pkg = "necst_ros3"
      name = "encoder_az_sim"
      type = "encoder_az_sim.py" />
  </node>

  <node pkg = "necst_ros3"
      name = "encoder_el_sim"
      type = "encoder_el_sim.py" />
  </node>

</launch>
