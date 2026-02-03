class_name VehicleController3D
extends RigidBody3D
## RigidBody-based vehicle that coordinates wheel physics and input.


## Toggles wheel debug rendering and force gizmos.
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
## GUIDE action used for throttle input.
@export var input_vehicle_acceleration: GUIDEAction
## GUIDE action used for steering input.
@export var input_vehicle_steering: GUIDEAction
## Shared vehicle settings resource.
@export var settings: VehicleControllerSettings

#@export_group("Nodes")
#@export var wheel_nodes: Array[VehicleController3DWheel] = []
# @export var front_right: VehicleController3DWheel
# @export var front_left: VehicleController3DWheel
# @export var back_right: VehicleController3DWheel
# @export var back_left: VehicleController3DWheel


## Lookup of wheels by key (F-L, F-R, etc.).
var wheels: Dictionary[String, VehicleController3DWheel] = {}
## Distance between front and rear axle centers.
var body_length: float
## Distance between left and right wheels on an axle.
var axle_length: float
## Linear velocity expressed in vehicle-local space.
var local_velocity: Vector3
## Tracks whether the initial ground position has been applied.
var initial_position_set: bool = false


## Initializes wheels and derives vehicle dimensions on ready.
func _ready() -> void:
	_initialize_wheels()
	_calculate_body_axle_length()


## Handles input-driven steering updates each frame.
func _physics_process(delta: float) -> void:
	if not _has_required_inputs():
		return
	if not initial_position_set:
		_try_set_initial_position()
		return

	# Take the vehicle's rotation out of its velocity.
	# For example, if local_velocity.x is positive, the vehicle is moving right. If negative, it's moving left.
	#local_velocity = global_transform.basis.inverse() * linear_velocity

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

## Attempts to snap the vehicle to the ground on first update.
func _try_set_initial_position() -> void:
	var start_pos: Vector3 = _query_initial_position()
	if start_pos.is_finite():
		global_position = start_pos
		initial_position_set = true


## Queries the ground beneath the vehicle to find a valid starting position.
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

## Calculates the Ackermann angle for a wheel on the given axle side.
func ackermann_angle(steering_rotation: float, axle_sign: float) -> float:
	return atan(body_length/(body_length/tan(steering_rotation)+(axle_sign*axle_length)/2.0))

#endregion


#region Initialization

## Finds wheel nodes, assigns roles, and initializes wheel state.
func _initialize_wheels() -> void:
	var wheel_nodes: Array[VehicleController3DWheel] = []
	wheel_nodes.assign(find_children("*", "VehicleController3DWheel"))

	if wheel_nodes.is_empty():
		push_error("No wheel nodes assigned to Vehicle.")
		return

	# Assuming the vehicle is facing in the Vector3.FORWARD (negative Z) direction, we can iterate over each wheel and determine what side it's on.
	# (From vehicle's perspective)
	#    Negative X -> Left of vehicle
	#    Positive X -> Right of vehicle
	#    Largest Z -> Furthest back
	#    Smallest Z -> Furthest front
	var left_wheels: Array[VehicleController3DWheel] = []
	var right_wheels: Array[VehicleController3DWheel] = []

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

	left_wheels.sort_custom(func(a: VehicleController3DWheel, b: VehicleController3DWheel) -> bool:
		return a.position.z < b.position.z
	)
	right_wheels.sort_custom(func(a: VehicleController3DWheel, b: VehicleController3DWheel) -> bool:
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
		wheels[wheel_key].debug = debug
		# Set wheel's front-back variable (and the is_powered variable)
		if "F" in wheel_key:
			if settings.drive_type == settings.DriveType.FWD or settings.drive_type == settings.DriveType.AWD:
				wheels[wheel_key].is_powered = true
			wheels[wheel_key].front_back = VehicleController3DWheel.FrontBackType.FRONT
		elif "B" in wheel_key:
			if settings.drive_type == settings.DriveType.RWD or settings.drive_type == settings.DriveType.AWD:
				wheels[wheel_key].is_powered = true
			wheels[wheel_key].front_back = VehicleController3DWheel.FrontBackType.BACK
		else:
			wheels[wheel_key].front_back = VehicleController3DWheel.FrontBackType.MIDDLE

		# Set wheel's left-right variable
		if "R" in wheel_key:
			wheels[wheel_key].left_right = VehicleController3DWheel.LeftRightType.RIGHT
		else:
			wheels[wheel_key].left_right = VehicleController3DWheel.LeftRightType.LEFT

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


## Validates that required settings, inputs, and wheels are available.
func _has_required_inputs() -> bool:
	if settings == null:
		push_error("Vehicle settings are missing.")
		return false
	if input_vehicle_acceleration == null or input_vehicle_steering == null:
		push_error("Vehicle input actions are missing.")
		return false
	if not wheels.has("F-R") or not wheels.has("F-L") or not wheels.has("B-R") or not wheels.has("B-L"):
		push_error("Vehicle wheels are not initialized.")
		return false
	return true

#endregion
