<launch>

  <node pkg = "necst_ros3"
    name = "antenna_az"
    type = "antenna_az.py" />

  <node pkg = "necst_ros3"
    name = "antenna_az_feedback"
    type = "antenna_az_feedback.py" />

  <node pkg = "necst_ros3"
    name = "antenna_az_mapper"
    type = "antenna_az_mapper.py" />

  <node pkg = "necst_ros3"
    name = "antenna_control"
    type = "antenna_control_sim.py" />

  <node pkg = "necst_ros3"
    name = "antenna_drive"
    type = "antenna_drive.py" />

  <node pkg = "necst_ros3"
    name = "antenna_el"
    type = "antenna_el.py" />

  <node pkg = "necst_ros3"
    name = "antenna_el_feedback"
    type = "antenna_el_feedback.py" />

  <node pkg = "necst_ros3"
    name = "antenna_el_mapper"
    type = "antenna_el_mapper.py" />

  <node pkg = "necst_ros3"
    name = "antenna_emergency"
    type = "antenna_emergency_sim.py" />

</launch>

