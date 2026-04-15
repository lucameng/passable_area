std_msgs/Header header
bool frame_partial
bool rear_dropout
uint16 sector_count
uint32 base_point_count
# Historical field name kept for compatibility. The counted samples follow the
# internal map-view cloud semantics even though the field still says odom.
uint32 odom_point_count
uint8[] sector_states
float32[] sector_coverage_confidence
