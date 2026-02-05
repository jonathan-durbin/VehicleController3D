class_name VehicleWheel
extends RayCast3D


enum FrontBackType {FRONT, BACK, MIDDLE}
enum LeftRightType {RIGHT, LEFT}


@export var spring_strength: float = 2000.0
@export var spring_rest_dist: float = 0.5:
	set(value):
		spring_rest_dist = value
		update_rest_dist()
@export var wheel_radius: float = 0.3


var debug: bool = false:
	set(value):
		debug = value
		if is_instance_valid(gizmo):
			gizmo.visible = debug
		for b in debug_boxes.values():
			if is_instance_valid(b):
				b.visible = debug
var parent: Vehicle
var global_contact_point: Vector3
var total_forces: Vector3
var wheel_mesh: MeshInstance3D
var is_powered: bool = false
var front_back: FrontBackType
var left_right: LeftRightType
## spring_damp_coefficient * (2.0 * sqrt(spring_strength * mass)).
## Where spring_damp_coefficient is between 0 and 1.
## More arcade-y: b/w 0.1 and 0.2. More realistic: b/w 0.2 and 1.0.
## Where the wheel mesh rests relative to the origin of the raycast, downwards.
## Calculated in initialize()
var spring_damping: float
var gizmo: Gizmo3D
var debug_boxes: Dictionary[String, MeshInstance3D] = {}


@onready var debug_box_scene: PackedScene = preload("uid://b0p5nm6aj735b")


func _physics_process(delta: float) -> void:
	total_forces = Vector3.ZERO
	force_raycast_update()

	if is_colliding():
		global_contact_point = get_collision_point()
		_set_debug_box("global_contact_point", global_contact_point, Color.ORANGE_RED)

		_handle_suspension(delta)
		_handle_acceleration()
	
	_handle_wheel_rotation(delta)

	if debug:
		gizmo.global_position = wheel_mesh.global_position
		gizmo.set_targetf("X", total_forces.x/parent.mass/5.0)
		gizmo.set_targetf("Y", total_forces.y/parent.mass/5.0)
		gizmo.set_targetf("Z", -total_forces.z/parent.mass/5.0)


func _handle_suspension(delta: float) -> void:
	var spring_up_dir: Vector3 = global_basis.y
	var spring_len: float = global_position.distance_to(global_contact_point) - wheel_radius
	var offset: float = spring_rest_dist - spring_len

	wheel_mesh.position.y = lerpf(wheel_mesh.position.y, -spring_len, delta * 20.0)

	var spring_force: float = spring_strength * offset

	var world_vel: Vector3 = _get_point_velocity(global_contact_point)
	var relative_vel: float = spring_up_dir.dot(world_vel)
	var damping_force: float = spring_damping * relative_vel

	total_forces.y = spring_force - damping_force

	var force_vector: Vector3 = total_forces.y * get_collision_normal()
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position

	_set_debug_box("suspension_force_offset_pos", wheel_mesh.global_position, Color("b0e952"))

	parent.apply_force(force_vector, force_offset_pos)


func _handle_acceleration() -> void:
	var forward_dir: Vector3 = -global_basis.z
	var speed: float = forward_dir.dot(parent.linear_velocity)
	var speed_ratio: float = speed / parent.max_speed
	var acceleration_ratio: float = parent.engine_power_curve.sample_baked(speed_ratio)
	
	if parent.input_accelerate.is_triggered():
		total_forces.z = parent.engine_power * parent.input_accelerate.value_axis_1d * float(is_powered)
	elif absf(speed) > 0.05 and not parent.input_accelerate.is_triggered():
		# Apply drag
		total_forces.z = -parent.deceleration * signf(speed)
	
	var force_vector: Vector3 = total_forces.z * forward_dir * acceleration_ratio
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position
	
	#var normal: Vector3 = get_collision_normal()
	#var projected_vector: Vector3 = (force_vector - normal * force_vector.dot(normal))

	if is_powered:
		_set_debug_box("accel_force_offset_pos", wheel_mesh.global_position+Vector3.ONE*0.01, Color("32a9f5"))

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
	spring_damping = parent.spring_damp_coefficient * (2.0 * sqrt(spring_strength * parent.mass))


func update_rest_dist() -> void:
	target_position.y = -(spring_rest_dist + wheel_radius)
	if is_instance_valid(wheel_mesh):
		wheel_mesh.position.y = -spring_rest_dist


func _get_point_velocity(point: Vector3) -> Vector3:
	return parent.linear_velocity + parent.angular_velocity.cross(point - parent.global_position)


func _handle_wheel_rotation(delta: float) -> void:
	var forward_dir: Vector3 = -global_basis.z
	var speed: float = forward_dir.dot(parent.linear_velocity)
	wheel_mesh.rotate_x(-speed * delta * 2.0 * PI * wheel_radius)
	

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
