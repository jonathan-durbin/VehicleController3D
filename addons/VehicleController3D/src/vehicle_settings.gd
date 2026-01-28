class_name VehicleControllerSettings
extends Resource
## Resource that defines tunable vehicle physics settings.


@export_group("Suspension and Wheels")
## Spring strength applied to front wheels.
@export var front_spring_strength: float = 500
## Spring strength applied to back wheels.
@export var back_spring_strength: float = 500
## For other wheels (not front or back, but in the middle)
@export var other_spring_strength: float = 500
## Used to reduce the amount by which the spring moves per unit of time.
## Higher values -> more syrupy
@export var spring_damp: float = 30
## How far below the wheel origin should the wheel be allowed to move
## i.e. when the car is in the air, how much should the wheels be offset in the (local) -Y direction?
## Basically, this is the maximum length of the wheel spring (This could be different for each wheel, if desired - might be fun)
@export var spring_rest_dist: float = 0.3
## Distance from the center of the wheel to the outside edge
@export var wheel_radius: float = 0.3
## Amount of vertical offset being applied to set the raycast form within the vehicle's body.
## Some small value like 0.5m should be a good default.
## I *think* the right idea is to offset such that the raycast starts from within the vehicle body.
@export var vertical_offset: float = 0.5
## Collision layer mask used for ground raycasts.
@export_flags_3d_physics var ground_layer: int


@export_group("Engine")
## Drive layout configuration for distributing power.
enum DriveType {
	FWD,
	RWD,
	AWD,
}
## Selected drive layout for the vehicle.
@export var drive_type := DriveType.FWD
## Peak engine power multiplier for drive force.
@export var engine_power: float = 35
## Curve describing available power by normalized speed.
@export var power_curve: CurveTexture
## Maximum forward speed used for normalization and limiting.
@export var max_forward_speed: float = 16
## Linear drag, attributable to rolling, moving parts, misc losses
@export var linear_drag: float = 10.0
## Aerodynamic drag, scales with v^2
@export var quadratic_drag: float = 1.0


@export_group("Steering")
## Maximum steering angle in degrees.
@export var steering_angle: float = 25
## Interpolation speed for steering response.
@export var steering_speed: float = 2


@export_group("Grip")
## X axis: % of tire lateral velocity
## Y axis: grip
## Grip curve for front tires.
@export var front_wheel_grip_curve: CurveTexture
## Grip curve for rear tires.
@export var rear_wheel_grip_curve: CurveTexture
## Grip curve for non-front/back tires.
@export var other_wheel_grip_curve: CurveTexture
## How much the wheels can slip before they're considered 100% slipping
@export var max_wheel_lateral_speed: float = 20
## Maximum lateral force allowed per wheel.
@export var max_lateral_force: float = 50
