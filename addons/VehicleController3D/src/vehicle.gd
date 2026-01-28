extends RigidBody3D
class_name Vehicle


@export var debug: bool = false:
	set(value):
		debug = value
		if wheels:
			for wheel in wheels.keys():
				wheels[wheel].debug = debug

@export_group("Settings")
#@export var input_vehicle_acceleration: String
#@export var input_vehicle_steering: String
#@export var settings: VehicleSettings
@export var input_vehicle_acceleration: GUIDEAction
@export var input_vehicle_steering: GUIDEAction
@export var settings: VehicleSettings

#@export_group("Nodes")
#@export var wheel_nodes: Array[VehicleWheel] = []
# @export var front_right: VehicleWheel
# @export var front_left: VehicleWheel
# @export var back_right: VehicleWheel
# @export var back_left: VehicleWheel


var wheels: Dictionary[String, VehicleWheel] = {}
var body_length: float
var axle_length: float
var local_velocity: Vector3
var initial_position_set: bool = false


# var steering_input: float
# var accel_input: float
# var wheels: Dictionary[String, CarWheel] = {}
# var body_length: float
# var axle_length: float
# var local_velocity: Vector3
# var initial_position_set: bool = false


func _ready() -> void:
	_initialize_wheels()
	_calculate_body_axle_length()


func _process(delta: float) -> void:
	if not initial_position_set:
		_try_set_initial_position()
		return

	# Take the vehicle's rotation out of its velocity.
	# For example, if local_velocity.x is positive, the vehicle is moving right. If negative, it's moving left.
	local_velocity = global_transform.basis.inverse() * linear_velocity

	# Between steering_angle and -steering_angle (before converting to radians)
	# Negate because positive Y rotation is counter-clockwise by default
	var steering_rotation = deg_to_rad(-input_vehicle_steering.value_axis_1d * settings.steering_angle) # TODO: express the negative in GUIDE instead of here(?)
	if not is_zero_approx(steering_rotation):
		wheels["F-R"].rotation.y = lerpf(
			wheels["F-R"].rotation.y,
			ackermann_angle(steering_rotation, 1.0),
			settings.steering_speed*delta
		)
		wheels["F-L"].rotation.y = lerpf(
			wheels["F-L"].rotation.y,
			ackermann_angle(steering_rotation, -1.0),
			settings.steering_speed*delta
		)
	else:
		wheels["F-L"].rotation.y = lerpf(wheels["F-L"].rotation.y, 0.0, settings.steering_speed*delta*1.5) # multiply by scaling factor to return to straight faster
		wheels["F-R"].rotation.y = lerpf(wheels["F-R"].rotation.y, 0.0, settings.steering_speed*delta*1.5)


#region Initial Position Calculation
func _try_set_initial_position() -> void:
	var start_pos: Vector3 = _query_initial_position()
	if start_pos.is_finite():
		global_position = start_pos
		initial_position_set = true


func _query_initial_position() -> Vector3:
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.create(
		global_position + Vector3.UP * 100.0,
		global_position - Vector3.UP * 100.0,
		settings.ground_layer
	)
	var result: Dictionary = space_state.intersect_ray(query)
	if not result.is_empty():
		return result.position + Vector3(0, 1, 0)
	else:
		return Vector3.INF

#endregion


#region Utility
## Calculates the Ackermann angle given wheel rotation and what side of the vehicle we're considering
func ackermann_angle(steering_rotation: float, axle_sign: float) -> float:
	return atan(body_length/(body_length/tan(steering_rotation)+(axle_sign*axle_length)/2.0))


## Returns a callable to be used in the Array.reduce method.
## Used when you want to find the key pointing to the largest value
## Expected dictionary data type should be Dictionary[Variant, float]
func reduce_max_key(d: Dictionary) -> Callable:
	return (
		func(acc, key):
		if acc == null:
			return key
		return key if d[key] > d[acc] else acc
	)


## Returns a callable to be used in the Array.reduce method.
## Used when you want to find the key pointing to the smallest value
## Expected dictionary data type should be Dictionary[Variant, float]
func reduce_min_key(d: Dictionary) -> Callable:
	return (
		func(acc, key):
		if acc == null:
			return key
		return key if d[key] < d[acc] else acc
	)

#endregion


#region Initialization
func _initialize_wheels() -> void:
	# Assuming the vehicle is facing in the Vector3.FORWARD (negative Z) direction, we can iterate over each wheel and determine what side it's on.
	# (From vehicle's perspective)
	#    Negative X -> Left of vehicle
	#    Positive X -> Right of vehicle
	#    Largest Z -> Furthest back
	#    Smallest Z -> Furthest front
	var wheel_nodes: Array[VehicleWheel] = []
	wheel_nodes.assign(find_children("*", "VehicleWheel"))
	var left_z: Dictionary[VehicleWheel, float] = {}
	var right_z: Dictionary[VehicleWheel, float] = {}
	for wheel in wheel_nodes:
		if is_equal_approx(signf(wheel.position.x), -1.0):
			left_z[wheel] = wheel.position.z
		else:
			right_z[wheel] = wheel.position.z

	# Set the four primary wheels
	wheels["B-L"] = left_z.keys().reduce(reduce_max_key(left_z))
	wheels["F-L"] = left_z.keys().reduce(reduce_min_key(left_z))
	wheels["B-R"] = right_z.keys().reduce(reduce_max_key(right_z))
	wheels["F-R"] = right_z.keys().reduce(reduce_min_key(right_z))

	# Fill in the rest of the wheels
	var wheel_num: int = 0
	for wheel in wheel_nodes:
		if wheel in wheels.values():
			continue
		var wheel_key: String = "M" + str(wheel_num)
		if is_equal_approx(signf(wheel.position.x), -1.0):
			wheel_key = wheel_key + "-L"
		else:
			wheel_key = wheel_key + "-R"
		wheels[wheel_key] = wheel
		wheel_num += 1

	# Sanity check
	if wheels.values().size() != wheel_nodes.size():
		push_error("Wheel array size mismatch!")

	# Set debug, is_powered, and other variables
	for wheel_key in wheels.keys():
		wheels[wheel_key].debug = debug
		# Set wheel's front-back variable (and the is_powered variable)
		if "F" in wheel_key:
			if settings.drive_type == settings.DriveType.FWD or settings.drive_type == settings.DriveType.AWD:
				wheels[wheel_key].is_powered = true
			wheels[wheel_key].front_back = VehicleWheel.FrontBackType.FRONT
		elif "B" in wheel_key:
			if settings.drive_type == settings.DriveType.RWD or settings.drive_type == settings.DriveType.AWD:
				wheels[wheel_key].is_powered = true
			wheels[wheel_key].front_back = VehicleWheel.FrontBackType.BACK
		else:
			wheels[wheel_key].front_back = VehicleWheel.FrontBackType.NONE

		# Set wheel's left-right variable
		if "R" in wheel_key:
			wheels[wheel_key].left_right = VehicleWheel.LeftRightType.RIGHT
		else:
			wheels[wheel_key].left_right = VehicleWheel.LeftRightType.LEFT

		wheels[wheel_key].initialize()


func _calculate_body_axle_length() -> void:
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

#endregion
