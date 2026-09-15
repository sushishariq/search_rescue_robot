# One victim (ArUco marker) in the map frame.
# Published per detection by sar_perception, and returned (merged) by the registry.
std_msgs/Header header       # frame_id = map; stamp = image time (detection) or last seen (registry)
int32 id                     # ArUco marker id
geometry_msgs/Point position # marker centre in map frame [m]
float32 range                # camera-to-marker distance of this / the best observation [m]
uint32 observations          # 1 for a raw detection; merged count in the registry
