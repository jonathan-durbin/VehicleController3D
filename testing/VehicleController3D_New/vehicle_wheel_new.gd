extends RayCast3D


var parent: RigidBody3D
var is_initialized: bool = false
var total_forces: Vector3 = Vector3.ZERO
## Initial position of the wheel. Suspension is trying to get back to this point
var wheel_rest_pos: Vector3
## How much the spring has been displaced
var spring_disp: float
var wheel_radius: float = 0.3
var global_collision_pos: Vector3
var spring_strength: float = 100.0
var previous_spring_compression: float = 0.0
var spring_damp: float = 50.0
var wheel_velocity: Vector3


@onready var wheel_mesh: MeshInstance3D = $WheelMesh
@onready var top_marker: Marker3D = $TopMarker
@onready var bottom_marker: Marker3D = $BottomMarker


func _physics_process(delta: float) -> void:
	#if not is_initialized: 
		#return
	
	total_forces = Vector3.ZERO
	wheel_velocity = get_velocity_at(wheel_mesh.global_position)
	if "FR" in name:
		pass
	
	if is_colliding():
		global_collision_pos = get_collision_point()
		_handle_suspension(delta)
		_handle_acceleration(delta)
		_handle_steering(delta)
		_update_wheel_height(delta, to_local(global_collision_pos).y + wheel_radius)
	else:
		_update_wheel_height(delta, wheel_rest_pos.y)


func initialize() -> void:
	parent = get_parent()
	wheel_rest_pos = wheel_mesh.position
	is_initialized = true
	

func _handle_suspension(_delta: float) -> void:
	var suspension_dir: Vector3 = global_basis.y
	var to_hit: Vector3 = global_collision_pos - global_position
	var spring_length: float = suspension_dir.dot(to_hit) # Project along suspension_dir

	var spring_compression: float = wheel_rest_pos.y - spring_length

	#var spring_velocity: float = (previous_spring_compression - spring_compression) / delta
	#previous_spring_compression = spring_compression
	
	var vel: float = suspension_dir.dot(wheel_velocity)

	#var damp_force: float = spring_damp * spring_velocity
	#var spring_force: float = spring_strength * spring_compression
	
	total_forces.y = (spring_strength * spring_compression) + (spring_damp * vel)
	
	parent.apply_force(
		total_forces.y * suspension_dir, 
		wheel_mesh.global_position - parent.global_position
	)
	
	
func _handle_acceleration(delta: float) -> void:
	pass
	
	
func _handle_steering(delta: float) -> void:
	pass
	
	
func _update_wheel_height(delta: float, height: float) -> void:
	wheel_mesh.position.y = lerpf(wheel_mesh.position.y, height, delta * 10.0)


## Calls PhysicsDirectBodyState3D.get_velocity_at_local_position(parent.to_local(point))
##    to get the velocity at the position of a global position, `point`.
func get_velocity_at(point: Vector3) -> Vector3:
	var vehicle_direct_state: PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(parent.get_rid())
	var velocity_at_point: Vector3 = vehicle_direct_state.get_velocity_at_local_position(parent.to_local(point))
	return velocity_at_point
