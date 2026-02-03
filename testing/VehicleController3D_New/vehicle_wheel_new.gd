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
## damp_coefficient * (2.0 * sqrt(spring_strength * mass)).
## Where damp_coefficient is between 0 and 1.
## More arcade-y: b/w 0.1 and 0.2. More realistic: b/w 0.2 and 1.0.
## Where the wheel mesh rests relative to the origin of the raycast, downwards.
## Calculated in initialize()
var spring_damping: float
var gizmo: Gizmo3D
var is_powered: bool = false
var front_back: FrontBackType
var left_right: LeftRightType
var debug_box: MeshInstance3D
var debug_boxes: Dictionary[String, MeshInstance3D] = {}
var debug_box_color: Color = Color(0.213, 0.196, 0.37, 1.0)


func _physics_process(delta: float) -> void:
	total_forces = Vector3.ZERO

	if is_colliding():
		global_contact_point = get_collision_point()
		_set_debug_box("global_contact_point", global_contact_point)

		_handle_suspension(delta)
		_handle_acceleration()

	if debug:
		gizmo.global_position = wheel_mesh.global_position
		gizmo.set_targetf("X", total_forces.x/parent.mass)
		gizmo.set_targetf("Y", total_forces.y/parent.mass)
		gizmo.set_targetf("Z", -total_forces.z/parent.mass)


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

	var force_vector: Vector3 = total_forces.y * spring_up_dir
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position
	
	_set_debug_box("suspension_force_offset_pos", parent.to_global(force_offset_pos))

	parent.apply_force(force_vector, force_offset_pos)


func _handle_acceleration() -> void:
	if not is_powered:
		return
	
	var forward_dir: Vector3 = -global_basis.z
	
	total_forces.z = parent.acceleration * parent.input_accelerate.value_axis_1d
	
	var force_vector: Vector3 = total_forces.z * forward_dir 
	var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position
	
	_set_debug_box("accel_force_offset_pos", parent.to_global(force_offset_pos))

	parent.apply_force(force_vector, force_offset_pos)



func initialize() -> void:
	parent = get_parent()

	var meshes: Array[MeshInstance3D]
	meshes.assign(find_children("*Wheel*", "MeshInstance3D"))
	wheel_mesh = meshes[meshes.find_custom(func(x: MeshInstance3D): return x.name.contains("Wheel"))]

	gizmo = Gizmo3D.new()
	add_child(gizmo)
	gizmo.visible = debug
	
	debug_box = MeshInstance3D.new()
	var b: BoxMesh = BoxMesh.new()
	b.size = Vector3(0.3, 0.3, 0.3)
	debug_box.mesh = b
	debug_box.visible = false
	var mat: StandardMaterial3D = StandardMaterial3D.new()
	mat.albedo_color = debug_box_color
	b.material = mat

	spring_damping = 0.1 * (2.0 * sqrt(spring_strength * parent.mass))

	update_rest_dist()


func update_rest_dist() -> void:
	target_position.y = -(spring_rest_dist + wheel_radius)
	if is_instance_valid(wheel_mesh):
		wheel_mesh.position.y = -spring_rest_dist


func _get_point_velocity(point: Vector3) -> Vector3:
	return parent.linear_velocity + parent.angular_velocity.cross(point - parent.global_position)


func _set_debug_box(s: String, pos: Vector3) -> void:
	if not debug:
		return
	if debug_boxes.has(s):
		debug_boxes[s].global_position = pos
	else:
		var b: MeshInstance3D = debug_box.duplicate()
		debug_box_color.h = wrapf(debug_box_color.h + 0.2, 0.0, 1.0) 
		b.mesh.material = StandardMaterial3D.new()
		b.mesh.material.albedo_color = debug_box_color
		debug_boxes[s] = b
		add_child(b)
		b.visible = debug
