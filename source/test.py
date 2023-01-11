import harfang as hg
from math import radians

hg.LoadPlugins()

plus = hg.GetPlus()
plus.RenderInit(640, 400)

plus.Mount("./")

scn = plus.NewScene()

cam = plus.AddCamera(scn,hg.Matrix4.TranslationMatrix(hg.Vector3(0, 1, -10)))
plus.AddLight(scn, hg.Matrix4.TranslationMatrix(hg.Vector3(6, 8, -6)))

ground = plus.AddPhysicPlane(scn, hg.Matrix4.TransformationMatrix(hg.Vector3(0, 0, 0), hg.Vector3(0, 0, 0)), 100,
								 100, 0, "assets/materials/green.mat")
ground[0].SetName("ground")
ground[1].SetType(hg.RigidBodyKinematic)


cube = plus.AddPhysicCube(scn, hg.Matrix4.TransformationMatrix(hg.Vector3(0, 2, 0),
																	   hg.Vector3(radians(0), 0, radians(0))), 1, 1,
								1, 1, "assets/materials/yellow.mat")
cube_node=cube[0]
cube_rigid = cube[1]
cube_node.SetName("tremplin")
#cube[1].SetType(hg.RigidBodyKinematic)



fps = hg.FPSController(0, 2, -10)

while not plus.IsAppEnded():
	delta_t = plus.UpdateClock()
	dts = hg.time_to_sec_f(delta_t)

	fps.UpdateAndApplyToNode(cam, delta_t)

	spring_friction = 40
	ray_pos_center = hg.Vec3(0,-0.6,0)
	rays_pos = [hg.Vector3(-0.5, -0.6, -0.5), hg.Vector3(0.5, -0.6, 0.5), hg.Vector3(-0.5, -0.6, 0.5), hg.Vector3(0.5, -0.6, -0.5)]
	mat = cube_node.GetTransform().GetWorld()
	cpos = mat.GetTranslation()
	ay = mat.GetY()
	ray_dir = ay * -1
	ray_max_dist = 1

	for i in range(2):
		rp = rays_pos[i]
		ray_pos = mat * rp
		#hit = physics.RaycastFirstHit(scene, ray_pos, ray_dir + ray_pos)

		ground_hit, ground_impact= scn.GetPhysicSystem().Raycast(ray_pos, ray_dir, 0x255,ray_max_dist)

		if ground_hit:
			v = scn.GetPhysicSystem().GetRigidBodyVelocity(cube_rigid, ray_pos).Reversed()
			v_dot_ground_n = hg.Dot(ground_impact.GetNormal(), v)
			if v_dot_ground_n > 0:
				v_bounce = ground_impact.GetNormal() * v_dot_ground_n
				cube_rigid.ApplyImpulse(v_bounce * spring_friction * dts, ray_pos)


	plus.UpdateScene(scn, delta_t)
	plus.Text2D(5, 5, "Move around with QSZD, left mouse button to look around")
	plus.Flip()
	plus.EndFrame()

plus.RenderUninit()