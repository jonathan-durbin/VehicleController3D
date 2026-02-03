extends RigidBody3D


@onready var vehicle_wheel_br: RayCast3D = $VehicleWheelBR
@onready var vehicle_wheel_fr: RayCast3D = $VehicleWheelFR
@onready var vehicle_wheel_bl: RayCast3D = $VehicleWheelBL
@onready var vehicle_wheel_fl: RayCast3D = $VehicleWheelFL


func _ready() -> void:
	for w in [vehicle_wheel_br, vehicle_wheel_bl, vehicle_wheel_fr, vehicle_wheel_fl]:
		w.initialize()
		
