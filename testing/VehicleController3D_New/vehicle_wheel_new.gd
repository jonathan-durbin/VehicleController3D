class_name VehicleWheel
extends RayCast3D


enum FrontBackType {FRONT, BACK, MIDDLE}
enum LeftRightType {RIGHT, LEFT}


var debug: bool = false:
	set(value):
		debug = value
		_update_debug_visibility()
var parent: Vehicle
## Point at which the ray detects an intersection, in global coordinates
var global_contact_point: Vector3
## Total forces being exerted on the vehicle from this wheel
var total_forces: Vector3
var wheel_mesh: MeshInstance3D
## Whether this wheel pushes the car forward
var is_powered: bool = false
## This wheel's front-back-middle type
var front_back: FrontBackType
## This wheel's left-right type
var left_right: LeftRightType
## spring_damp_coefficient * (2.0 * sqrt(spring_strength * mass)).
## Where spring_damp_coefficient is between 0 and 1.
## More arcade-y: b/w 0.1 and 0.2. More realistic: b/w 0.2 and 1.0.
## Calculated in initialize()
var spring_damping: float
var is_slipping: bool = false
var wheel_velocity: Vector3
var gizmo: Gizmo3D
var debug_boxes: Dictionary[String, MeshInstance3D] = {}


@onready var debug_box_scene: PackedScene = preload("uid://b0p5nm6aj735b")


func _physics_process(delta: float) -> void:
	total_forces = Vector3.ZERO
	wheel_velocity = _get_point_velocity(wheel_mesh.global_position)

	if is_colliding():
		global_contact_point = get_collision_point()
		_set_debug_box("global_contact_point", global_contact_point, Color.ORANGE_RED)

		_handle_suspension()
		_handle_acceleration()
		_handle_traction()

		wheel_mesh.position.y = lerpf(
			wheel_mesh.position.y,
			-maxf(0.0, global_position.distance_to(global_contact_point) - parent.wheel_radius_map[front_back]),
			delta * 20.0
		)
	else:
		wheel_mesh.position.y = lerpf(
			wheel_mesh.position.y,
			-parent.spring_rest_dist_map[front_back],
			delta * 20.0
		)

	_handle_wheel_rotation(delta)

	if debug:
		gizmo.global_position = wheel_mesh.global_position
		gizmo.set_targetf("X", total_forces.x/parent.mass/5.0)
		gizmo.set_targetf("Y", total_forces.y/parent.mass/5.0)
		gizmo.set_targetf("Z", -total_forces.z/parent.mass/5.0)


func _handle_suspension() -> void:
	var spring_up_dir: Vector3 = global_basis.y
	var spring_len: float = maxf(0.0, global_position.distance_to(global_contact_point) - parent.wheel_radius_map[front_back])
	var offset: float = parent.spring_rest_dist_map[front_back] - spring_len

	var spring_force: float = parent.spring_strength_map[front_back] * offset

	var relative_vel: float = spring_up_dir.dot(wheel_velocity)
	var damping_force: float = spring_damping * relative_vel

	total_forces.y = spring_force - damping_force

	var force_vector: Vector3 = total_forces.y * get_collision_normal()
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position

	_set_debug_box("suspension_force_offset_pos", wheel_mesh.global_position, Color("b0e952"))

	parent.apply_force(force_vector, force_offset_pos)


func _handle_acceleration() -> void:
	var forward_dir: Vector3 = -global_basis.z
	var forward_speed: float = forward_dir.dot(parent.linear_velocity)
	var speed_ratio: float = forward_speed / parent.max_speed
	var available_power: float = parent.engine_power_curve.sample(
		clampf(absf(speed_ratio), 0.0, 1.0)
	)

	# if parent.input_accelerate.is_triggered():
	total_forces.z = (
		parent.engine_power
		* parent.input_accelerate.value_axis_1d
		* available_power
		* float(is_powered)
	)

	var force_vector: Vector3 = total_forces.z * forward_dir
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position

	# Drag
	var forward_tire_speed: float = forward_dir.dot(wheel_velocity)
	# Could adjust based on the type of ground
	var z_drag: float = 0.05
	var z_drag_force: float = forward_tire_speed * z_drag * _get_wheel_weight()
	var z_drag_force_vector: Vector3 = -forward_dir * z_drag_force

	total_forces.z += z_drag_force
	force_vector += z_drag_force_vector

	# Optional: project the acceleration vector along the contact surface
	#var normal: Vector3 = get_collision_normal()
	#var projected_vector: Vector3 = (force_vector - normal * force_vector.dot(normal))

	if is_powered:
		_set_debug_box("accel_force_offset_pos", wheel_mesh.global_position+Vector3.ONE*0.01, Color("32a9f5"))

	parent.apply_force(force_vector, force_offset_pos)


func _handle_traction() -> void:
	var steer_side_dir: Vector3 = global_basis.x
	var x_speed: float = steer_side_dir.dot(wheel_velocity)

	var grip_factor: float = clampf(absf(x_speed/wheel_velocity.length()), 0.0, 1.0)
	# var grip_factor: float = absf(x_speed/wheel_velocity.length())
	var x_traction: float = parent.grip_curve_map[front_back].sample(grip_factor)

	#if not parent.input_handbrake.is_triggered() and grip_factor < 0.2:
		#is_slipping = false
#
	#if parent.input_handbrake.is_triggered():
		#is_slipping = true
		#x_traction = 0.01
	#elif is_slipping:
		#x_traction = 0.1

	total_forces.x = -x_speed * x_traction * _get_wheel_weight()

	var force_vector: Vector3 = total_forces.x * steer_side_dir
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position

	parent.apply_force(force_vector, force_offset_pos)


func initialize() -> void:
	parent = get_parent()

	var meshes: Array[MeshInstance3D]
	meshes.assign(find_children("*Wheel*", "MeshInstance3D"))
	wheel_mesh = meshes[meshes.find_custom(func(x: MeshInstance3D): return x.name.contains("Wheel"))]

	gizmo = Gizmo3D.new()
	add_child(gizmo)
	gizmo.visible = debug

	calculate_spring_damping()

	update_rest_dist()


func calculate_spring_damping() -> void:
	spring_damping = parent.spring_damp_coefficient * (2.0 * sqrt(parent.spring_strength_map[front_back] * parent.mass))


func update_rest_dist() -> void:
	target_position.y = -(
		parent.spring_rest_dist_map[front_back]
		+ parent.wheel_radius_map[front_back]
		+ parent.over_extend_map[front_back]
	)
	if is_instance_valid(wheel_mesh):
		wheel_mesh.position.y = -parent.spring_rest_dist_map[front_back]


func _get_point_velocity(point: Vector3) -> Vector3:
	return parent.linear_velocity + parent.angular_velocity.cross(point - parent.global_position)


func _get_wheel_weight() -> float:
	var gravity: float = absf(parent.get_gravity().y)
	return (parent.mass * gravity) / float(parent.wheels.size()) # Number of wheels


func _handle_wheel_rotation(delta: float) -> void:
	var forward_dir: Vector3 = -global_basis.z
	var speed: float = forward_dir.dot(parent.linear_velocity)
	wheel_mesh.rotate_x((-speed * delta) / parent.wheel_radius_map[front_back])


func _set_debug_box(s: String, pos: Vector3, c: Color) -> void:
	if not debug:
		return
	if debug_boxes.has(s):
		debug_boxes[s].global_position = pos
	else:
		var b: MeshInstance3D = debug_box_scene.instantiate()
		b.material_override.albedo_color = c
		debug_boxes[s] = b
		add_child(b)
		b.visible = debug
		b.global_position = pos


func _update_debug_visibility() -> void:
	if is_instance_valid(gizmo):
		gizmo.visible = debug
	for b in debug_boxes.values():
		if is_instance_valid(b):
			b.visible = debug
