# -*-coding:Utf-8 -*

# ===========================================================

#              - HARFANG® 3D - www.harfang3d.com

#                    - Python tutorial -

#                   Simulation raycast car

# ===========================================================

import harfang as hg
from math import radians, sqrt
import time

# ========================================================================================================
#               Car class
# ========================================================================================================

class Car:
	def __init__(self, name, plus, scene, start_position: hg.Vector3, start_rotation=hg.Vector3.Zero):
		self.name = name
		self.chassis_node = scene.GetNode("car_body")
		self.thrust = scene.GetNode("thrust")
		self.wheels = self.get_wheels(plus, scene)
		self.chassis_rigid, self.collisions_boxes = self.get_collisions(scene)

		self.local_rays = self.get_rays()
		self.ray_dir = None
		self.wheels_ray = self.wheels[0].GetObject().GetLocalMinMax().mx.y
		self.ray_max_dist = self.wheels_ray + 0.2

		self.wheels_rot_speed = [0] * 4
		self.ground_hits = [False] * 4
		self.ground_impacts = [None] * 4

		self.mass = 0
		self.density = 0
		self.spring_friction = 0
		self.tires_reaction = 0
		self.tires_adhesion = 0
		self.front_angle = 0
		self.front_angle_max = 45

		self.setup()

		self.start_position = start_position
		self.start_rotation = start_rotation
		self.chassis_node.GetTransform().SetPosition(start_position)

		plus.UpdateScene(scene, plus.UpdateClock())

	def setup(self, mass=1500, spring_friction=30, tires_reaction=26, tires_adhesion=2.4, linear_damping=0.2,
			  angular_damping=0.2):
		self.chassis_rigid.SetAngularDamping(linear_damping)
		self.chassis_rigid.SetLinearDamping(angular_damping)
		self.spring_friction = spring_friction
		self.tires_reaction = tires_reaction
		self.tires_adhesion = tires_adhesion
		self.set_mass(mass)

	def set_mass(self, mass):
		volumes = []
		total_volume = 0
		for colbox in self.collisions_boxes:
			dimensions = colbox.GetDimensions()
			volume = dimensions.x * dimensions.y * dimensions.z
			total_volume += volume
			volumes.append(volume)
		self.density = mass / total_volume / 1000
		for i in range(len(self.collisions_boxes)):
			self.collisions_boxes[i].SetMass(volumes[i] / total_volume * mass)
		self.mass = mass

	def set_density(self, density=0.3):
		self.density = density
		self.mass = 0
		for colbox in self.collisions_boxes:
			dimensions = colbox.GetDimensions()
			box_mass = dimensions.x * dimensions.y * dimensions.z * self.density * 1000
			colbox.SetMass(box_mass)
			self.mass += box_mass

	def get_collisions(self, scene):
		rigid = hg.RigidBody()
		rigid.SetType(hg.RigidBodyDynamic)
		self.chassis_node.AddComponent(rigid)
		collisions_nodes = scene.GetNodes("col_shape")
		collisions_boxes = []
		for col_shape in collisions_nodes:
			colbox = hg.BoxCollision()
			collisions_boxes.append(colbox)
			obj = col_shape.GetObject()
			bounds = obj.GetLocalMinMax()
			dimensions = bounds.mx - bounds.mn
			pos = col_shape.GetTransform().GetPosition() + bounds.mn + dimensions * 0.5
			colbox.SetDimensions(dimensions)
			colbox.SetMatrix(hg.Matrix4.TranslationMatrix(pos))
			self.chassis_node.AddComponent(colbox)
			scene.RemoveNode(col_shape)
		return rigid, collisions_boxes

	def reset(self):
		self.chassis_rigid.ResetWorld(hg.Matrix4.TransformationMatrix(self.start_position, self.start_rotation))

	def get_rays(self):
		rays = []
		for wheel in self.wheels:
			rays.append(wheel.GetTransform().GetPosition())
		return rays

	def get_wheels(self, plus, scene):
		wheels = []
		for n in range(4):
			wheel = scene.GetNode("wheel_" + str(n))
			wheels.append(wheel)
		return wheels

	def turn(self, angle):
		self.front_angle = max(min(self.front_angle + angle, self.front_angle_max), -self.front_angle_max)
		self.thrust.GetTransform().SetRotation(hg.Vector3(0, radians(self.front_angle), 0))

	def accelerate(self, value):
		f = 0
		for i in range(2):
			if self.ground_hits[i]:
				f += 0.5
		pos = self.thrust.GetTransform().GetWorld().GetTranslation()
		dir = self.thrust.GetTransform().GetWorld().GetZ()
		#self.chassis_rigid.ApplyForce(dir * self.mass * f * value, pos)
		self.chassis_rigid.ApplyImpulse(dir *  f * value * (1/60) , pos)

	def brake(self, value):
		f = 0
		for i in range(4):
			if self.ground_hits[i]:
				f += 0.25
		v = self.chassis_rigid.GetLinearVelocity()
		value *= min(v.Len(), 1)
		#self.chassis_rigid.ApplyLinearForce(v.Normalized() * self.mass * f * -value)
		pos = self.thrust.GetTransform().GetWorld().GetTranslation()
		self.chassis_rigid.ApplyImpulse(v.Normalized() * (1/60) * f * -value,pos)

	def update_kinetic(self, scene, dts):
		self.chassis_rigid.SetIsSleeping(False)
		self.ray_dir = self.chassis_node.GetTransform().GetWorld().GetY().Reversed()
		for i in range(4):
			self.update_wheel_physic(i, scene, dts)

	def update_wheel_physic(self, id, scene, dts):
		wheel = self.wheels[id]
		mat = self.chassis_node.GetTransform().GetWorld()  # Ray position in World space
		ray_pos = mat * self.local_rays[id]
		self.ground_hits[id], self.ground_impacts[id] = scene.GetPhysicSystem().Raycast(ray_pos, self.ray_dir, 0x255,
																						self.ray_max_dist)

		if self.ground_hits[id]:
			v = scene.GetPhysicSystem().GetRigidBodyVelocity(self.chassis_rigid, ray_pos).Reversed()
			hit_distance = (self.ground_impacts[id].GetPosition() - ray_pos).Len()

			# Spring bounce:
			v_dot_ground_n = hg.Dot(self.ground_impacts[id].GetNormal(), v)
			if v_dot_ground_n > 0:
				v_bounce = self.ground_impacts[id].GetNormal() * v_dot_ground_n
				self.chassis_rigid.ApplyImpulse(v_bounce * self.spring_friction *dts, ray_pos)

			# Tire/Ground reaction:
			wheel_reaction = sqrt(self.ray_max_dist - hit_distance) * self.tires_reaction
			self.chassis_rigid.ApplyForce(self.ground_impacts[id].GetNormal() * wheel_reaction * self.mass / 4, ray_pos)
			#self.chassis_rigid.ApplyImpulse(self.ground_impacts[id].GetNormal() * wheel_reaction  * dts, ray_pos)

			# Wheel lateral friction:
			x_axis = wheel.GetTransform().GetWorld().GetX()
			proj = hg.Dot(x_axis, v)
			v_lat = x_axis * proj
			self.chassis_rigid.ApplyImpulse(v_lat * self.tires_adhesion *dts , ray_pos)

			# Adjust wheel on the ground.
			wheel_p = wheel.GetTransform().GetPosition()
			wheel_p.y = self.local_rays[id].y - hit_distance + self.wheels_ray
			wheel.GetTransform().SetPosition(wheel_p)

			# Wheel rotation:
			z_axis = hg.Cross(x_axis, self.ray_dir).Normalized()
			vlin = hg.Dot(z_axis, v)  # Linear speed (along Z axis)
			self.wheels_rot_speed[id] = (vlin / self.wheels_ray)
		else:
			self.wheels_rot_speed[id] *= 0.95  # Wheel slow-down

		rot = wheel.GetTransform().GetRotation()
		rot.x += self.wheels_rot_speed[id] * dts
		if id == 0 or id == 1:
			rot.y = radians(self.front_angle)
		wheel.GetTransform().SetRotation(rot)

	def get_parent_node(self):
		return self.chassis_node


# ==================================================================================================
#                       Camera follow
# ==================================================================================================

follow_distance = 0
follow_altitude = 0
fps_ref = 60
target_point = hg.Vector3(0, 0, 0)
target_node = None
wall_jump_k = 0
v_wall_jump = None


def setup_camera_follow(targetNode: hg.Node, targetPoint: hg.Vector3, distance=25, altitude=5, jump_force=20, fps=60):
	global target_point, target_node, follow_distance, follow_altitude, fps_ref, v_wall_jump, wall_jump_k
	target_point = targetPoint
	target_node = targetNode
	follow_distance = distance
	follow_altitude = altitude
	v_wall_jump = hg.Vector3(0, 0, 0)
	fps_ref = fps
	wall_jump_k = jump_force


def RangeAdjust(value, oldmin, oldmax, newmin, newmax):
	return (value - oldmin) / (oldmax - oldmin) * (newmax - newmin) + newmin


def update_target_point(dts, inertia=0.075):
	global target_point
	v = target_node.GetTransform().GetPosition() - target_point
	target_point += v * inertia * fps_ref * dts


def update_follow_translation(camera: hg.Node, dts, inertia=0.025):
	global v_wall_jump
	trans = camera.GetTransform()
	camera_pos = camera.GetTransform().GetPosition()
	target_pos = target_node.GetTransform().GetPosition()

	# Wall
	v = target_pos - camera_pos
	target_dir = v.Normalized()
	target_dist = v.Len()

	hit, impact = scene.GetPhysicSystem().Raycast(trans.GetPosition(), target_dir, 255, target_dist)
	if hit and impact.GetNode() != target_node:
		v_wall_jump.y += wall_jump_k * dts
	else:
		v_wall_jump *= pow(0.9 ,fps_ref * dts)

	y_alt = follow_altitude - (camera_pos.y - target_pos.y)
	v_wall_jump.y += y_alt * dts

	camera_pos += v_wall_jump * dts
	v_trans = target_dir * (target_dist - follow_distance)

	camera.GetTransform().SetPosition(camera_pos + v_trans * inertia * fps_ref * dts)


def update_follow_direction(camera: hg.Node):
	v = target_point - camera.GetTransform().GetPosition()
	camera.GetTransform().SetRotationMatrix(
		camera.GetTransform().GetWorld().GetRotationMatrix().LookAt(v, hg.Vector3.Up))


def update_camera_follow(camera: hg.Node, dts):
	global target_point, target_node
	update_target_point(dts)
	update_follow_direction(camera)
	update_follow_translation(camera, dts)

# ==============================================================================================
#               Displays
# ==============================================================================================

class Editor:
	courbe = []
	ymin = 0
	ymax = 0
	start_courbe = False

	nfps = [0] * 60
	nfps_i = 0

	@classmethod
	def maj_courbe(cls, y):
		if not cls.start_courbe:
			cls.ymin = y
			cls.ymax = y
			cls.start_courbe = True
		else:
			if y < cls.ymin:
				ymin = y
			if y > cls.ymax:
				cls.ymax = y
		cls.courbe.append(y)

	@classmethod
	def affiche_courbe(cls, plus, resolution: hg.Vector2):
		num = len(cls.courbe)
		if num > 10:
			x_step = resolution.x / (num - 1)
			x1 = 0
			x2 = x_step
			y1 = (cls.courbe[0] - cls.ymin) / (cls.ymax - cls.ymin) * resolution.y
			for i in range(num - 1):
				y2 = (cls.courbe[i + 1] - cls.ymin) / (cls.ymax - cls.ymin) * resolution.y
				plus.Line2D(x1, y1, x2, y2, hg.Color.Yellow, hg.Color.Yellow)
				x1 = x2
				x2 += x_step
				y1 = y2

	@classmethod
	def get_2d(cls, camera, renderer, point3d: hg.Vector3):
		f, pos = hg.Project(camera.GetTransform().GetWorld(), camera.GetCamera().GetZoomFactor(),
							renderer.GetAspectRatio(), point3d)
		if f:
			return hg.Vector2(pos.x, 1 - pos.y)
		else:
			return None

	@classmethod
	def affiche_vecteur(cls, plus, camera, position, direction, c1=hg.Color.Yellow, c2=hg.Color.Red, unitaire=True):
		if unitaire:
			position_b = position + direction.Normalized()
		else:
			position_b = position + direction
		pA = cls.get_2d(camera, plus.GetRenderer(), position)
		pB = cls.get_2d(camera, plus.GetRenderer(), position_b)
		if pA is not None and pB is not None:
			plus.Line2D(pA.x * resolution.x, pA.y * resolution.y, pB.x * resolution.x, pB.y * resolution.y, c1, c2)

	@classmethod
	def affiche_repere(cls, plus, camera, position: hg.Vector3, repere: hg.Matrix3):
		cls.affiche_vecteur(plus, camera, position, repere.GetX(), hg.Color.White, hg.Color.Red)
		cls.affiche_vecteur(plus, camera, position, repere.GetY(), hg.Color.White, hg.Color.Green)
		cls.affiche_vecteur(plus, camera, position, repere.GetZ(), hg.Color.White, hg.Color.Blue)

	@classmethod
	def draw_car_vectors(cls, car, plus, scene):
		mat = car.chassis_node.GetTransform().GetWorld()
		mat_thrust = car.thrust.GetTransform().GetWorld()
		cls.affiche_repere(plus, scene.GetNode("Camera"), car.chassis_node.GetTransform().GetPosition(),
						   mat.GetRotationMatrix())
		cls.affiche_repere(plus, scene.GetNode("Camera"), mat_thrust.GetTranslation(), mat_thrust.GetRotationMatrix())

		for n in range(4):
			ray_pos = mat * car.local_rays[n]
			v = scene.GetPhysicSystem().GetRigidBodyVelocity(car.chassis_rigid, ray_pos)
			cls.affiche_vecteur(plus, scene.GetNode("Camera"), ray_pos, v, hg.Color.Blue, hg.Color.Red, False)
			if car.ground_hits[n]:
				cls.affiche_vecteur(plus, scene.GetNode("Camera"), ray_pos,
									car.ground_impacts[n].GetPosition() - ray_pos, hg.Color.Yellow, hg.Color.Red, False)

	@classmethod
	def display_fps(cls, plus,dt,resolution: hg.Vector2):
		cls.nfps[cls.nfps_i] = 1 / dt
		cls.nfps_i = (cls.nfps_i + 1) % len(cls.nfps)
		nf = 0
		for ne in cls.nfps:
			nf += ne
		nf = nf / len(cls.nfps)
		plus.Text2D(0.01 * resolution.x, 0.95 * resolution.y, "FPS: %d" % int(nf), 20,
		                hg.Color.Yellow)

# ==============================================================================================
#               Functions
# ==============================================================================================


def init_scene(plus):
	scene = plus.NewScene()
	camera = plus.AddCamera(scene, hg.Matrix4.TranslationMatrix(hg.Vector3(20, 10, 10)))
	camera.SetName("Camera")
	init_lights(plus, scene)

	plus.LoadScene(scene, "assets/car_big_wheeler/car_big_wheeler.scn")

	while not scene.IsReady():  # Wait until scene is ready
		plus.UpdateScene(scene, plus.UpdateClock())

	car = Car("Kubolid", plus, scene, hg.Vector3(0, 3, 0))

	# Ground:
	ground = plus.AddPhysicPlane(scene, hg.Matrix4.TransformationMatrix(hg.Vector3(0, 0, 0), hg.Vector3(0, 0, 0)), 100,
								 100, 0, "assets/materials/green.mat")
	ground[0].SetName("ground")
	ground[1].SetType(hg.RigidBodyKinematic)

	cube = plus.AddPhysicCube(scene, hg.Matrix4.TransformationMatrix(hg.Vector3(0, -2.5, -10),
																	   hg.Vector3(radians(30), 0, radians(10))), 10, 10,
								10, 0, "assets/materials/green.mat")
	cube[0].SetName("tremplin")
	cube[1].SetType(hg.RigidBodyKinematic)

	return scene, car


def init_lights(plus, scene):
	# Main light:
	ligth_sun = plus.AddLight(scene, hg.Matrix4.RotationMatrix(hg.Vector3(radians(22), radians(-45), 0)),
							  hg.LightModelLinear)
	ligth_sun.SetName("Sun")
	ligth_sun.GetLight().SetDiffuseColor(hg.Color(255. / 255., 255. / 255., 255. / 255., 1.))

	ligth_sun.GetLight().SetShadow(hg.LightShadowMap)  # Active les ombres portées
	ligth_sun.GetLight().SetShadowRange(50)

	ligth_sun.GetLight().SetDiffuseIntensity(1.)
	ligth_sun.GetLight().SetSpecularIntensity(1.)

	# Sky ligth:
	ligth_sky = plus.AddLight(scene, hg.Matrix4.RotationMatrix(hg.Vector3(radians(54), radians(135), 0)),
							  hg.LightModelLinear)
	ligth_sky.SetName("SkyLigth")
	ligth_sky.GetLight().SetDiffuseColor(hg.Color(103. / 255., 157. / 255., 141. / 255., 1.))
	ligth_sky.GetLight().SetDiffuseIntensity(0.9)

def gui_interface(scene, car: Car, display_vectors: bool):
	global low_fps
	if hg.ImGuiBegin("Car settings"):
		hg.ImGuiText("Arrows, Space : Car controls")
		f, display_vectors = hg.ImGuiCheckbox("Show vectors", display_vectors)
		if f:
			scene.GetPhysicSystem().SetDebugVisuals(display_vectors)
		f, low_fps = hg.ImGuiCheckbox("Low FPS", low_fps)
		if hg.ImGuiButton("Backspace : Reset car"):
			car.reset()
	hg.ImGuiEnd()
	return display_vectors



def car_control(plus, car, dts):
	if plus.KeyDown(hg.KeyUp):
		car.accelerate(10)
	if plus.KeyDown(hg.KeyDown):
		car.accelerate(-10)
	if plus.KeyDown(hg.KeySpace):
		car.brake(10)
	if plus.KeyDown(hg.KeyLeft):
		car.turn(-150 * dts)
	if plus.KeyDown(hg.KeyRight):
		car.turn(150 * dts)
	if plus.KeyPress (hg.KeyBackspace):
		car.reset()


# ==================================================================================================

#                                   Program start here

# ==================================================================================================

# Display settings
resolution = hg.Vector2(1600, 900)
antialiasing = 4
screenMode = hg.Windowed

display_vectors = False
low_fps = False

# System setup
plus = hg.GetPlus()
hg.LoadPlugins()
plus.Mount("./")

# Run display
plus.RenderInit(int(resolution.x), int(resolution.y), antialiasing, screenMode)
plus.SetBlend2D(hg.BlendAlpha)
# plus.SetBlend3D(hg.BlendAlpha)

# Setup scene:
scene, car = init_scene(plus)

# check if we use VR
flag_vr = False
if flag_vr:
	try:
		openvr_frame_renderer = hg.CreateFrameRenderer("VR")
		if openvr_frame_renderer.Initialize(plus.GetRenderSystem()):
			scene.GetRenderableSystem().SetFrameRenderer(openvr_frame_renderer)
			print("!! Use VR")
		else:
			openvr_frame_renderer = None
			print("!! No VR detected")
	except:
		print("!! No VR detected")
		openvr_frame_renderer = None
else:
	openvr_frame_renderer = None


camera = scene.GetNode("Camera")
setup_camera_follow(car.get_parent_node(), car.get_parent_node().GetTransform().GetPosition())

# -----------------------------------------------
#                   Main loop
# -----------------------------------------------

car.chassis_rigid.ResetWorld(
	hg.Matrix4.TransformationMatrix(car.chassis_node.GetTransform().GetPosition(), hg.Vector3(0, 0, 0)))

while not plus.KeyDown(hg.KeyEscape) and not plus.IsAppEnded():
	delta_t = plus.UpdateClock()
	dts = hg.time_to_sec_f(delta_t)

	display_vectors = gui_interface(scene, car, display_vectors)

	car.update_kinetic(scene, dts)
	car_control(plus, car, dts)

	if display_vectors:
		Editor.draw_car_vectors(car, plus, scene)
		Editor.display_fps(plus,dts,resolution)

	plus.UpdateScene(scene, delta_t)
	update_camera_follow(camera, dts)

	if low_fps:
		time.sleep(0.05)

	plus.Flip()
	plus.EndFrame()

plus.RenderUninit()
