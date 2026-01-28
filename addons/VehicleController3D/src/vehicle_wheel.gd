class_name VehicleWheel
extends RayCast3D
## Raycast vehicle wheel node that handles suspension, traction, and visuals.

## Minimum delta used to avoid division by zero in physics calculations.
const MIN_DELTA: float = 0.0001

## Wheel position relative to vehicle length.
enum FrontBackType {
	FRONT,
	BACK,
	NONE,
}
## Wheel position relative to vehicle width.
enum LeftRightType {
	LEFT,
	RIGHT,
}

## Amount of vertical offset being applied.
## Some small value like 0.5m should be a good default.
## I *think* the right idea is to offset such that the raycast starts from within the vehicle body.
const VERTICAL_OFFSET: float = 0.5

## Enables debug visuals such as gizmos and force boxes.
var debug: bool = false:
	set(value):
		debug = value
		if is_instance_valid(debug_box):
			debug_box.visible = debug
		if is_instance_valid(gizmo):
			gizmo.visible = debug
## Set by Vehicle to indicate front/back placement.
## None type allows for vehicles to have middle wheels (tank-like vehicles)
var front_back: FrontBackType = FrontBackType.NONE
## Set by Vehicle to indicate left/right placement.
var left_right: LeftRightType
## Set by Vehicle to indicate whether the wheel receives drive force.
var is_powered: bool = false
## Mesh instance used to animate wheel visuals.
var wheel_meshinstance: MeshInstance3D
## Portion of the lateral velocity (x axis) to reduce by each frame.
var grip: float
## Used to calculate how quickly the suspension is displacing.
var previous_spring_length: float = 0.0
## Strength of the spring. Calculated and set in initialize().
var spring_strength: float
## The curve used to sample the amount of traction the wheels have. Set in initialize().
var grip_curve: Curve
## Total forces applied to the car from this wheel.
var total_forces: Vector3
## Position from which forces are applied.
## Set to the collision point plus the wheel radius,
##    placing it in the center of each wheel
var force_position: Vector3
## Velocity of the wheel local to the car, calculated every physics tick.
var velocity_at_wheel: Vector3
## Debug mesh for visualizing applied force position. Created in initialize().
var debug_box: MeshInstance3D
## Debug gizmo for displaying force vectors. Created in initialize().
var gizmo: Gizmo3D
## This is meant to be considered the "true" global position of the wheel, since the raycast is
## It's good to bump the raycast up by a bit so that if the Vehicle bottoms out, the ground is still detected.
# var global_position_offset: Vector3
## The original position of the raycast after applying the offset.
## Set as the initial position of the raycast (before applying the above offset).
## The wheel mesh cannot be placed above this point.
var wheel_origin: Vector3

## Parent vehicle that owns this wheel.
var parent: Vehicle


## Performs per-physics-tick wheel simulation and visual updates.
func _physics_process(delta: float) -> void:
	# global_position_offset = to_global(position - Vector3(0.0, VERTICAL_OFFSET, 0.0))
	velocity_at_wheel = get_velocity_at(to_global(wheel_origin))
	total_forces = Vector3.ZERO
	if is_colliding():
		#print("colliding")
		var collision_point := get_collision_point() # Global coordinates
		force_position = Vector3(
			collision_point.x,
			collision_point.y + parent.settings.wheel_radius, # up from the ground
			collision_point.z
		)

		if debug:
			debug_box.global_position = force_position

		handle_suspension(delta, collision_point)
		handle_acceleration(delta, collision_point)
		handle_steering(delta, collision_point)

		set_wheel_height(collision_point.y + parent.settings.wheel_radius, delta)
		rotate_wheel(delta)

	else:
		#print("not colliding")
		set_wheel_height(wheel_origin.y - parent.settings.spring_rest_dist, delta)

	if debug:
		gizmo.set_targetf("X", total_forces.x/20.0)
		gizmo.set_targetf("Y", total_forces.y/40.0)
		gizmo.set_targetf("Z", total_forces.z/20.0)


## Initializes wheel resources, raycast setup, and derived settings.
func initialize() -> void:
	# Set up debug box
	debug_box = MeshInstance3D.new()
	debug_box.mesh = BoxMesh.new()
	debug_box.mesh.size = Vector3(0.1, 0.1, 0.1)
	add_child(debug_box)
	debug_box.owner = owner
	debug_box.visible = debug
	# Set up debug gizmo
	gizmo = Gizmo3D.new()
	add_child(gizmo)
	gizmo.visible = debug

	parent = get_parent()

	# Find single mesh instance child (assuming there is only one, which might be fine)
	for child in find_children("*", "MeshInstance3D"):
		if "wheel" in child.name.to_lower():
			wheel_meshinstance = child
			break
	if wheel_meshinstance == null:
		push_warning("VehicleWheel mesh instance not found; visual wheel updates will be skipped.")

	# Set up raycast
	collision_mask = parent.settings.ground_layer
	# Set the max distance the raycast should look
	target_position = Vector3(0.0, -(VERTICAL_OFFSET + parent.settings.spring_rest_dist + parent.settings.wheel_radius), 0.0)

	# At this point, front_back and left_right have been set, so set the spring strength and grip curve
	spring_strength = (
		parent.settings.front_spring_strength if front_back == FrontBackType.FRONT
		else parent.settings.back_spring_strength if front_back == FrontBackType.BACK
		else parent.settings.other_spring_strength
	)
	grip_curve = (
		parent.settings.front_wheel_grip_curve.curve if front_back == FrontBackType.FRONT
		else parent.settings.rear_wheel_grip_curve.curve if front_back == FrontBackType.BACK
		else parent.settings.other_wheel_grip_curve.curve
	)

	# Apply vertical offset to fix issue with bottoming out
	wheel_origin = position
	position.y += VERTICAL_OFFSET
	# global_position_offset = to_global(position - Vector3(0.0, VERTICAL_OFFSET, 0.0))
	for child in get_children():
		child.position.y -= VERTICAL_OFFSET


#region Visuals - Wheel Mesh Position and Rotation
## Smoothly sets the wheel mesh height toward the suspension target.
func set_wheel_height(value: float, delta: float) -> void:
	if wheel_meshinstance == null:
		return
	wheel_meshinstance.position.y = lerpf(wheel_meshinstance.position.y, value, 20.0 * delta)


## Rotates the wheel mesh based on vehicle speed and direction.
func rotate_wheel(delta: float):
	if wheel_meshinstance == null:
		return
	var dir: Vector3 = parent.basis.z
	var rotation_dir: float = 1 if parent.linear_velocity.dot(dir) > 0.0 else -1
	wheel_meshinstance.rotate_x(rotation_dir * parent.linear_velocity.length() * delta)

#endregion


#region Suspension
## Applies suspension forces based on collision distance.
func handle_suspension(delta: float, collision_point: Vector3) -> void:
	var safe_delta := maxf(delta, MIN_DELTA)
	# Direction the suspension force will be applied
	var suspension_dir: Vector3 = global_basis.y
	var distance: float = collision_point.distance_to(to_global(wheel_origin))

	var spring_length: float = clampf(
		distance - parent.settings.wheel_radius, # Take the wheel radius out of the equation
		0.0, parent.settings.spring_rest_dist # Spring length should only be between 0.0 and spring_rest_dist
	)

	var spring_velocity: float = (previous_spring_length - spring_length) / safe_delta # Movement per time
	previous_spring_length = spring_length

	var damp_force: float = parent.settings.spring_damp * spring_velocity
	var spring_force: float = (
		spring_strength # Calculated once in initialize
		* (parent.settings.spring_rest_dist - spring_length)
	)

	total_forces.y = spring_force + damp_force
	# only for debugging - isolates one wheel
	#if front_back == FrontBackType.FRONT and left_right == LeftRightType.RIGHT:
		#print(to_global(wheel_origin), " | ", collision_point, " | ", distance, " | ", spring_length, " | ", spring_velocity, " | ", damp_force, " | ", spring_force)
		#print(suspension_dir)

	parent.apply_force(
		# Even if the car is leaning, we only want to apply a vertical spring force
		# Not entirely accurate, but results in greater stability
		suspension_dir * total_forces.y, # * Vector3.UP,
		# Must be offset from the body origin in global coordinates
		force_position - parent.global_position
	)
#endregion


#region Acceleration
## Applies drive and drag forces for powered wheels.
func handle_acceleration(_delta: float, _collision_point: Vector3) -> void:
	if not is_powered:
		return

	var accel_dir = -global_basis.z
	var forward_speed: float = accel_dir.dot(velocity_at_wheel)

	var max_speed := maxf(parent.settings.max_forward_speed, 0.001)
	var normalized_speed: float = clampf(
		absf(forward_speed) / max_speed,
		0.0, 1.0
	)

	# Negative because forward is in the -Z direction
	var throttle: float = -parent.input_vehicle_acceleration.value_axis_1d
	var available_power: float = parent.settings.power_curve.curve.sample(normalized_speed) * throttle

	var drive_force: float = available_power * parent.settings.engine_power

	# Calculate resistive forces
	var speed_abs: float = absf(forward_speed)
	var speed_sign: float = signf(forward_speed)
	var drag_force: float = (
		speed_sign
		* (
			parent.settings.linear_drag * speed_abs
			+ parent.settings.quadratic_drag * forward_speed * forward_speed
		)
	)

	total_forces.z = drive_force - drag_force
	parent.apply_force(accel_dir * total_forces.z, force_position - parent.global_position)
#endregion


#region Steering
## Applies lateral grip forces to counteract slip.
func handle_steering(delta: float, collision_point: Vector3) -> void:
	var safe_delta := maxf(delta, MIN_DELTA)
	var axle_dir: Vector3 = global_basis.x
	var lateral_speed: float = axle_dir.dot(velocity_at_wheel)

	# How much to counteract lateral speed
	grip = grip_curve.sample(clampf(
		absf(lateral_speed)/parent.settings.max_wheel_lateral_speed,
		0.0, 1.0
	))

	var velocity_change: float = -lateral_speed * grip
	var accel: float = velocity_change / safe_delta # Divide by time to get acceleration (m/s/s)

	total_forces.x = clampf(
		accel * parent.mass,
		-parent.settings.max_lateral_force,
		parent.settings.max_lateral_force
	)
	parent.apply_force(axle_dir * total_forces.x, collision_point - parent.global_position)
#endregion


#region Utility
## Calls PhysicsDirectBodyState3D.get_velocity_at_local_position(point - parent.global_position)
##    to get the velocity at the position of `point`.
func get_velocity_at(point: Vector3) -> Vector3:
	var car_direct_state: PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(parent.get_rid())
	var velocity_at_point: Vector3 = car_direct_state.get_velocity_at_local_position(point - parent.global_position)
	return velocity_at_point
#endregion
