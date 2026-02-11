extends RigidBody3D
class_name SBVWheelNub

var target_pos: Vector3

@onready var parent: Node3D = get_parent()


func _physics_process(delta: float) -> void:
	var up_dir: Vector3 = basis.y
	var offset: float = position.distance_to(target_pos)
	var spring_force: float = parent.spring_strength * offset
	apply_central_force(spring_force * up_dir)


#func _handle_suspension() -> void:
	#var spring_up_dir: Vector3 = global_basis.y
	#var spring_len: float = maxf(0.0, global_position.distance_to(global_contact_point) - parent.wheel_radius_map[front_back])
	#var offset: float = parent.spring_rest_dist_map[front_back] - spring_len
#
	#var spring_force: float = parent.spring_strength_map[front_back] * offset
#
	#var relative_vel: float = spring_up_dir.dot(wheel_velocity)
	#var damping_force: float = spring_damping * relative_vel
#
	#total_forces.y = spring_force - damping_force
#
	#var force_vector: Vector3 = total_forces.y * get_collision_normal()
	#var force_offset_pos: Vector3 = wheel_mesh.global_position - parent.global_position
#
	#_set_debug_box("suspension_force_offset_pos", wheel_mesh.global_position, Color("b0e952"))
#
	#parent.apply_force(force_vector, force_offset_pos)
