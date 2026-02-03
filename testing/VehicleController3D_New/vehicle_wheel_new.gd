class_name VehicleWheel
extends RayCast3D


@export var spring_strength: float = 2000.0
@export var spring_rest_dist: float = 0.5:
	set(value):
		spring_rest_dist = value
		update_rest_dist() 
@export var wheel_radius: float = 0.3


var debug: bool
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


func _physics_process(delta: float) -> void:
	total_forces = Vector3.ZERO

	if is_colliding():
		global_contact_point = get_collision_point()

		_handle_suspension(delta)


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
	var force_offset_pos: Vector3 = global_contact_point - parent.global_position
	
	parent.apply_force(force_vector, force_offset_pos)


func initialize() -> void:
	parent = get_parent()
	
	var meshes: Array[MeshInstance3D]
	meshes.assign(find_children("*Wheel*", "MeshInstance3D"))
	wheel_mesh = meshes[meshes.find_custom(func(x: MeshInstance3D): return x.name.contains("Wheel"))]
	
	spring_damping = 0.1 * (2.0 * sqrt(spring_strength * parent.mass))
	
	update_rest_dist()


func update_rest_dist() -> void:
	target_position.y = -(spring_rest_dist + wheel_radius)
	wheel_mesh.position.y = -spring_rest_dist


func _get_point_velocity(point: Vector3) -> Vector3:
	return parent.linear_velocity + parent.angular_velocity.cross(point - parent.global_position)
