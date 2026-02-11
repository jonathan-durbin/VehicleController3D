class_name VehicleController3D
extends RigidBody3D
## RigidBody-based vehicle that coordinates wheel physics and input.


enum DriveType {AWD, FWD, RWD}

@export var global_ctx: GUIDEMappingContext # TODO: handle in InputManager global

@export var debug: bool = false:
	set(value):
		debug = value
		for wheel in wheels.values():
			if is_instance_valid(wheel):
				wheel.debug = debug

@export var wheel_radius_map: Dictionary[VehicleController3DWheel.FrontBackType, float] = {
	VehicleController3DWheel.FrontBackType.FRONT: 0.3,
	VehicleController3DWheel.FrontBackType.BACK: 0.3,
	VehicleController3DWheel.FrontBackType.MIDDLE: 0.3,
}

## How much the raycast should over-extend below the ground.
## Setting this above zero will pull the vehicle towards the ground.
@export var over_extend_map: Dictionary[VehicleController3DWheel.FrontBackType, float] = {
	VehicleController3DWheel.FrontBackType.FRONT: 0.1,
	VehicleController3DWheel.FrontBackType.BACK: 0.1,
	VehicleController3DWheel.FrontBackType.MIDDLE: 0.1,
}

@export var spring_strength_map: Dictionary[VehicleController3DWheel.FrontBackType, float] = {
	VehicleController3DWheel.FrontBackType.FRONT: 2000.0,
	VehicleController3DWheel.FrontBackType.BACK: 2000.0,
	VehicleController3DWheel.FrontBackType.MIDDLE: 2000.0,
}

## How far down the raycast should the wheel mesh rest when the raycast is not colliding.
@export var spring_rest_dist_map: Dictionary[VehicleController3DWheel.FrontBackType, float] = {
	VehicleController3DWheel.FrontBackType.FRONT: 0.5,
	VehicleController3DWheel.FrontBackType.BACK: 0.5,
	VehicleController3DWheel.FrontBackType.MIDDLE: 0.5,
}

## spring_damp_coefficient * (2.0 * sqrt(spring_strength * mass)).
## Where spring_damp_coefficient is between 0 and 1.
## More arcade-y: b/w 0.1 and 0.2. More realistic: b/w 0.2 and 1.0.
## Where the wheel mesh rests relative to the origin of the raycast, downwards.
## Actual spring damping is calculated in VehicleController3DWheel.initialize()
@export_range(0.0, 1.0, 0.001) var spring_damp_coefficient: float = 0.1:
	set(value):
		spring_damp_coefficient = value
		for wheel in wheels.values():
			if is_instance_valid(wheel):
				wheel.calculate_spring_damping()

@export_range(0.0, 1000.0, 1.0) var engine_power: float = 300.0
@export var drive_type: DriveType = DriveType.RWD
@export var max_speed: float = 10.0 # m/s
@export var engine_power_curve: Curve

@export var tire_turn_speed_deg: float = 50.0
@export var tire_max_turn_deg: float = 25.0
@export var grip_curve_map: Dictionary[VehicleController3DWheel.FrontBackType, Curve] = {
	VehicleController3DWheel.FrontBackType.FRONT: null,
	VehicleController3DWheel.FrontBackType.BACK: null,
	VehicleController3DWheel.FrontBackType.MIDDLE: null,
}

@export var input_accelerate: GUIDEAction
@export var input_steer: GUIDEAction
@export var input_handbrake: GUIDEAction
@export_flags_3d_physics var ground_layer: int


## Lookup of wheels by key (F-L, F-R, etc.).
## If there are any middle wheels, those would be M1-R, M1-L, etc. In order from front to back.
var wheels: Dictionary[String, VehicleController3DWheel] = {}
## Distance between front and rear axle centers.
var body_length: float
## Distance between left and right wheels on an axle.
var axle_length: float
var grounded: bool = false


## Initializes wheels and derives vehicle dimensions on ready.
func _ready() -> void:
	GUIDE.enable_mapping_context(global_ctx)
	_initialize_wheels()
	_calculate_body_axle_length()


func _physics_process(delta: float) -> void:
	_update_center_of_mass()
	_handle_steering_rotation(delta)


#region Physics Process

func _update_center_of_mass() -> void:
	grounded = false
	for wheel in wheels:
		if wheels[wheel].is_colliding():
			grounded = true
	center_of_mass = Vector3.ZERO if grounded else Vector3.DOWN * 0.5


func _handle_steering_rotation(delta: float) -> void:
	var turn_input: float = input_steer.value_axis_1d * deg_to_rad(tire_max_turn_deg)

	if input_steer.is_triggered():
		wheels["F-L"].rotation.y = (
			# clampf(
				lerpf(
					wheels["F-L"].rotation.y,
					_ackermann_angle(turn_input, 1.0),
					deg_to_rad(tire_turn_speed_deg) * delta
				)
			# 	-deg_to_rad(tire_max_turn_deg), deg_to_rad(tire_max_turn_deg)
			# )
		)
		wheels["F-R"].rotation.y = (
			# clampf(
				lerpf(
					wheels["F-R"].rotation.y,
					_ackermann_angle(turn_input, -1.0),
					deg_to_rad(tire_turn_speed_deg) * delta
				)
			# 	-deg_to_rad(tire_max_turn_deg), deg_to_rad(tire_max_turn_deg)
			# )
		)
	else:
		wheels["F-L"].rotation.y = lerpf(
			wheels["F-L"].rotation.y,
			0.0, deg_to_rad(tire_turn_speed_deg) * delta * 1.5 # Return to 0.0 slightly faster
		)
		wheels["F-R"].rotation.y = lerpf(
			wheels["F-R"].rotation.y,
			0.0, deg_to_rad(tire_turn_speed_deg) * delta * 1.5
		)

#endregion


#region Util

## Calculates the Ackermann angle for a wheel on the given axle side.
func _ackermann_angle(steering_rotation: float, axle_sign: float) -> float:
	return atan(body_length/(body_length/tan(steering_rotation)+(axle_sign*axle_length)/2.0))


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

	left_wheels.sort_custom(
		func(a: VehicleController3DWheel, b: VehicleController3DWheel) -> bool:
		return a.position.z < b.position.z
	)
	right_wheels.sort_custom(
		func(a: VehicleController3DWheel, b: VehicleController3DWheel) -> bool:
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

	# Set debug, is_powered, left/right, and front/back variables
	for wheel_key in wheels.keys():
		wheels[wheel_key].debug = debug
		# Set wheel's front-back variable (and the is_powered variable)
		if "F" in wheel_key:
			if drive_type == DriveType.FWD or drive_type == DriveType.AWD:
				wheels[wheel_key].is_powered = true
			wheels[wheel_key].front_back = VehicleController3DWheel.FrontBackType.FRONT
		elif "B" in wheel_key:
			if drive_type == DriveType.RWD or drive_type == DriveType.AWD:
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
		push_error("Can't calculate body axle length, at least four wheels need to be defined")
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

#endregion
