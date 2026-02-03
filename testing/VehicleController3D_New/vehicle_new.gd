class_name Vehicle
extends RigidBody3D


@export var debug: bool = false

## Lookup of wheels by key (F-L, F-R, etc.).
var wheels: Dictionary[String, VehicleWheel] = {}
## Distance between front and rear axle centers.
var body_length: float
## Distance between left and right wheels on an axle.
var axle_length: float


## Initializes wheels and derives vehicle dimensions on ready.
func _ready() -> void:
	_initialize_wheels()
	_calculate_body_axle_length()




## Finds wheel nodes, assigns roles, and initializes wheel state.
func _initialize_wheels() -> void:
	var wheel_nodes: Array[VehicleWheel] = []
	wheel_nodes.assign(find_children("*", "VehicleWheel"))

	if wheel_nodes.is_empty():
		push_error("No wheel nodes assigned to Vehicle.")
		return

	# Assuming the vehicle is facing in the Vector3.FORWARD (negative Z) direction, we can iterate over each wheel and determine what side it's on.
	# (From vehicle's perspective)
	#    Negative X -> Left of vehicle
	#    Positive X -> Right of vehicle
	#    Largest Z -> Furthest back
	#    Smallest Z -> Furthest front
	var left_wheels: Array[VehicleWheel] = []
	var right_wheels: Array[VehicleWheel] = []

	for wheel in wheel_nodes:
		if is_zero_approx(wheel.position.x):
			push_warning("Wheel is centered on X; defaulting to right side: %s" % wheel.name)
		if wheel.position.x < 0.0:
			left_wheels.append(wheel)
		else:
			right_wheels.append(wheel)

	if left_wheels.is_empty() or right_wheels.is_empty():
		push_error("Wheel nodes must include both left and right wheels.")
		return
	if left_wheels.size() < 2 or right_wheels.size() < 2:
		push_error("Wheel nodes must include at least two wheels per side.")
		return

	left_wheels.sort_custom(func(a: VehicleWheel, b: VehicleWheel) -> bool:
		return a.position.z < b.position.z
	)
	right_wheels.sort_custom(func(a: VehicleWheel, b: VehicleWheel) -> bool:
		return a.position.z < b.position.z
	)

	# Set the four primary wheels
	wheels["F-L"] = left_wheels.front()
	wheels["B-L"] = left_wheels.back()
	wheels["F-R"] = right_wheels.front()
	wheels["B-R"] = right_wheels.back()

	# Fill in the rest of the wheels
	var wheel_num: int = 0
	for idx in range(1, left_wheels.size() - 1):
		wheels["M" + str(wheel_num) + "-L"] = left_wheels[idx]
		wheel_num += 1
	for idx in range(1, right_wheels.size() - 1):
		wheels["M" + str(wheel_num) + "-R"] = right_wheels[idx]
		wheel_num += 1

	# Sanity check
	if wheels.values().size() != wheel_nodes.size():
		push_error("Wheel array size mismatch!")

	# Set debug, is_powered, and other variables
	for wheel_key in wheels.keys():
	# 	wheels[wheel_key].debug = debug
	# 	# Set wheel's front-back variable (and the is_powered variable)
	# 	if "F" in wheel_key:
	# 		if settings.drive_type == settings.DriveType.FWD or settings.drive_type == settings.DriveType.AWD:
	# 			wheels[wheel_key].is_powered = true
	# 		wheels[wheel_key].front_back = VehicleWheel.FrontBackType.FRONT
	# 	elif "B" in wheel_key:
	# 		if settings.drive_type == settings.DriveType.RWD or settings.drive_type == settings.DriveType.AWD:
	# 			wheels[wheel_key].is_powered = true
	# 		wheels[wheel_key].front_back = VehicleWheel.FrontBackType.BACK
	# 	else:
	# 		wheels[wheel_key].front_back = VehicleWheel.FrontBackType.MIDDLE

	# 	# Set wheel's left-right variable
	# 	if "R" in wheel_key:
	# 		wheels[wheel_key].left_right = VehicleWheel.LeftRightType.RIGHT
	# 	else:
	# 		wheels[wheel_key].left_right = VehicleWheel.LeftRightType.LEFT

		wheels[wheel_key].initialize()


## Calculates body and axle lengths using wheel positions.
func _calculate_body_axle_length() -> void:
	if not wheels.has("F-R") or not wheels.has("F-L") or not wheels.has("B-R") or not wheels.has("B-L"):
		return
	body_length = ((
			wheels["F-R"].global_position.distance_to(wheels["B-R"].global_position) +
			wheels["F-L"].global_position.distance_to(wheels["B-L"].global_position)
		) / 2.0
	)
	axle_length = ((
			wheels["F-R"].global_position.distance_to(wheels["F-L"].global_position) +
			wheels["B-R"].global_position.distance_to(wheels["B-L"].global_position)
		) / 2.0
	)
