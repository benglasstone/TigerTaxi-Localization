define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

# Robot model:
# footprint (counter-clockwise): [-0.5, -0.6096], [1.9384, -0.6096], [1.9384, 0.6096], [-0.5, 0.6096]
# center of rotation: [0, 0.5]
# wheelbase: 1.4384

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 2.4384 1.2192 1.7653 ] # This models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0].
  # correct center of rotation:
  origin [ 0.74295 0.0 0.0 0.0]
  gui_nose 1
  color "orange"

  # kinematics
  drive "car"
  wheelbase 1.4384 # distance between both axles
  
  # spawn sensors
  laser(pose [ 1.4859 0.0 0.0 0.0 ])
)
