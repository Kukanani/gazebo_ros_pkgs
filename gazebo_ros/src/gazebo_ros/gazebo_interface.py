#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo
# Authors:
#   Adam Allevato
#   Dave Coleman: spawn_sdf_model_client, spawn_urdf_model_client,
#                 set_model_configuration_client

import rospy
import time

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from xml.etree.ElementTree import Element, SubElement, tostring


class GazeboInterface:
    def __init__(self, gazebo_namespace=None):
        if gazebo_namespace is None:
            gazebo_namespace = '/gazebo'
        self.gazebo_namespace = gazebo_namespace

    def pause_physics(self, ns=None):
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns + '/pause_physics')
        try:
            pause = rospy.ServiceProxy(ns + '/pause_physics', Empty)
            pause()
        except rospy.ServiceException as e:
            rospy.logerr("couldn't pause physics: " + str(e))
            return

    def unpause_physics(self, ns=None):
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns + '/unpause_physics')
        try:
            unpause = rospy.ServiceProxy(ns + '/unpause_physics', Empty)
            unpause()
        except rospy.ServiceException as e:
            rospy.logerr("couldn't unpause physics: " + str(e))
            return

    def get_model_state(self, model_name, ns=None):
        """
        Get the state of a model in the world
        :param model_name: name of the model for which to get the state
        :return: a gazebo_msgs/ModelState message with the object's information.
        """
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns + '/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy(ns + '/get_model_state',
                                                 GetModelState)
            state = get_model_state(model_name=model_name)
        except rospy.ServiceException as e:
            rospy.logerr(
                "couldn't get model state " + model_name + ": " + str(e))
            return None
        return state

    def set_model_state(self, model_name, xyz=None, quat=None, lin_vel=None,
                        ang_vel=None, ns=None):
        """
        Set a model's state. Any parameters not provided will be kept at their
        current values. All arguments should be provided as array-likes (list
        or numpy arrays).

        :param model_name:
        :param xyz:
        :param quat:
        :param lin_vel:
        :param ang_vel:
        :param ns:
        :return:
        """
        if ns is None:
            ns = self.gazebo_namespace

        rospy.wait_for_service(ns + '/get_model_state')
        rospy.wait_for_service(ns + '/set_model_state')
        try:
            get_model_state = rospy.ServiceProxy(ns + '/get_model_state',
                                                 GetModelState)
            state = get_model_state(model_name=model_name)
        except rospy.ServiceException as e:
            rospy.logerr(
                "couldn't get model state " + model_name + ": " + str(e))
            rospy.logerr(
                "couldn't get the model state, so won't set the model state")
            return

        if xyz is None:
            xyz = state.pose.position
        else:
            xyz = Point(xyz[0], xyz[1], xyz[2])
        if quat is None:
            quat = state.pose.orientation
        else:
            quat = Quaternion(quat[0], quat[1], quat[2], quat[3])
        if lin_vel is None:
            lin_vel = state.twist.linear
        else:
            lin_vel = Vector3(lin_vel[0], lin_vel[1], lin_vel[2])
        if ang_vel is None:
            ang_vel = state.twist.angular
        else:
            ang_vel = Vector3(ang_vel[0], ang_vel[1], ang_vel[2])

        try:
            set_model_state = rospy.ServiceProxy(ns + '/set_model_state',
                                                 SetModelState)
            set_model_state(
                model_state=ModelState(model_name=model_name,
                                       pose=Pose(position=xyz,
                                                 orientation=quat),
                                       twist=Twist(linear=lin_vel,
                                                   angular=ang_vel)))
        except rospy.ServiceException as e:
            rospy.logerr(
                "couldn't set model state " + model_name + ": " + str(e))

    def delete_model(self, model_name, ns=None):
        if ns is None:
            ns = self.gazebo_namespace

        rospy.wait_for_service(ns + '/delete_model')
        try:
            delete_model = rospy.ServiceProxy(ns + '/delete_model', DeleteModel)
            delete_model(model_name)
        except rospy.ServiceException as e:
            rospy.logerr("couldn't delete model " + model_name + ": " + str(e))

    def reset_world(self, ns=None):
        if ns is None:
            ns = self.gazebo_namespace

        rospy.wait_for_service(ns + '/reset_world')
        try:
            reset_world = rospy.ServiceProxy(ns + '/reset_world', Empty)
            reset_world()
        except rospy.ServiceException as e:
            rospy.logerr("couldn't reset world: " + str(e))

    def set_physics_properties(self, time_step=None,
                               max_update_rate=None, gravity=None, ns=None):
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns + '/get_physics_properties')
        rospy.wait_for_service(ns + '/set_physics_properties')

        try:
            get_physics_properties = \
                rospy.ServiceProxy(ns + '/get_physics_properties',
                                   GetPhysicsProperties)
            props = get_physics_properties()
        except rospy.ServiceException as e:
            rospy.logerr('couldn\'t get physics properties while preparing '
                         'to set physics properties: ' + str(e))
            return
        if time_step is None:
            time_step = props.time_step
        if max_update_rate is None:
            max_update_rate = props.max_update_rate
        if gravity is None:
            gravity = props.gravity
        elif isinstance(gravity, list):
            gravity = Vector3(gravity[0], gravity[1], gravity[2])

        try:
            set_physics_properties = \
                rospy.ServiceProxy(ns + '/set_physics_properties',
                                   SetPhysicsProperties)
            set_physics_properties(time_step=time_step,
                                   max_update_rate=max_update_rate,
                                   gravity=gravity,
                                   ode_config=props.ode_config)
        except rospy.ServiceException as e:
            rospy.logerr('couldn\'t set physics properties: ' + str(e))

    def spawn_box(self, model_name, dim_x=1.0, dim_y=1.0, dim_z=1.0, mass=1.0,
                  xyz=None, rpy=None, material=None,
                  mu1=0.5, mu2=0.5, physics="ode", ns=None):
        if xyz is None:
            xyz = [0.0, 0.0, 0.0]
        if rpy is None:
            rpy = [0.0, 0.0, 0.0]
        if material is None:
            material = 'Gazebo/White'
        if ns is None:
            ns = self.gazebo_namespace
        if physics != 'ode':
            rospy.logwarn('The SDF generator currently generates physics'
                          'information for ODE only, other physics engines'
                          'have not been implemented yet.')

        ixx = mass/12*(dim_y*dim_y+dim_z*dim_z)
        iyy = mass/12*(dim_x*dim_x+dim_z*dim_z)
        izz = mass/12*(dim_y*dim_y+dim_x*dim_x)

        size_str = str(dim_x) + ' ' + str(dim_y) + ' ' + str(dim_z)

        sdf_t = Element('sdf', {'version': '1.4'})
        model_t = SubElement(sdf_t, 'model', {'name': model_name})
        pose_t = SubElement(model_t, 'pose')
        pose_t.text = ' '.join([str(x) for x in xyz]) + ' ' + \
                      ' '.join([str(x) for x in rpy])
        link_t = SubElement(model_t, 'link', {'name': model_name})

        inertial_t = SubElement(link_t, 'inertial')
        mass_t = SubElement(inertial_t, 'mass')
        mass_t.text = str(mass)
        inertia_t = SubElement(inertial_t, 'inertia')
        SubElement(inertia_t, 'ixx').text = str(ixx)
        SubElement(inertia_t, 'ixy').text = '0'
        SubElement(inertia_t, 'ixz').text = '0'
        SubElement(inertia_t, 'iyy').text = str(iyy)
        SubElement(inertia_t, 'iyz').text = '0'
        SubElement(inertia_t, 'izz').text = str(izz)

        collision_t = SubElement(link_t, 'collision',
                                 {'name': model_name+'_collision'})
        geometry_t = SubElement(collision_t, 'geometry')
        box_t = SubElement(geometry_t, 'box')
        size_t = SubElement(box_t, 'size')
        size_t.text = size_str
        surface_t = SubElement(collision_t, 'surface')
        friction_t = SubElement(surface_t, 'friction')
        ode_t = SubElement(friction_t, 'ode')
        SubElement(ode_t, 'mu').text = str(mu1)
        SubElement(ode_t, 'mu2').text = str(mu2)

        visual_t = SubElement(link_t, 'visual', {'name': model_name+'_visual'})
        geometry_t = SubElement(visual_t, 'geometry')
        box_t = SubElement(geometry_t, 'box')
        size_t = SubElement(box_t, 'size')
        size_t.text = size_str

        self._add_material_tag(material, visual_t)

        self.spawn_sdf_model_client(model_name, tostring(sdf_t), ns=ns)

    def spawn_cylinder(self, model_name, radius=0.5, length=1.0, mass=1.0,
                       xyz=None, rpy=None, material=None,
                       mu1=0.5, mu2=0.5, physics="ode", ns=None):
        if xyz is None:
            xyz = [0.0, 0.0, 0.0]
        if rpy is None:
            rpy = [0.0, 0.0, 0.0]
        if material is None:
            material = 'Gazebo/White'
        if ns is None:
            ns = self.gazebo_namespace
        if physics != 'ode':
            rospy.logwarn('The SDF generator currently generates physics'
                          'information for ODE only, other physics engines'
                          'have not been implemented yet.')

        ixx = mass/12*(3*radius*radius+length*length)
        iyy = mass/12*(3*radius*radius+length*length)
        izz = mass/2*radius*radius

        sdf_t = Element('sdf', {'version': '1.4'})
        model_t = SubElement(sdf_t, 'model', {'name': model_name})
        pose_t = SubElement(model_t, 'pose')
        pose_t.text = ' '.join([str(x) for x in xyz]) + ' ' + \
                      ' '.join([str(x) for x in rpy])
        link_t = SubElement(model_t, 'link', {'name': model_name})

        inertial_t = SubElement(link_t, 'inertial')
        mass_t = SubElement(inertial_t, 'mass')
        mass_t.text = str(mass)
        inertia_t = SubElement(inertial_t, 'inertia')
        SubElement(inertia_t, 'ixx').text = str(ixx)
        SubElement(inertia_t, 'ixy').text = '0'
        SubElement(inertia_t, 'ixz').text = '0'
        SubElement(inertia_t, 'iyy').text = str(iyy)
        SubElement(inertia_t, 'iyz').text = '0'
        SubElement(inertia_t, 'izz').text = str(izz)

        collision_t = SubElement(link_t, 'collision',
                                 {'name': model_name+'_collision'})
        geometry_t = SubElement(collision_t, 'geometry')
        cylinder_t = SubElement(geometry_t, 'cylinder')
        radius_t = SubElement(cylinder_t, 'radius')
        radius_t.text = str(radius)
        length_t = SubElement(cylinder_t, 'length')
        length_t.text = str(length)
        surface_t = SubElement(collision_t, 'surface')
        friction_t = SubElement(surface_t, 'friction')
        ode_t = SubElement(friction_t, 'ode')
        SubElement(ode_t, 'mu').text = str(mu1)
        SubElement(ode_t, 'mu2').text = str(mu2)

        visual_t = SubElement(link_t, 'visual', {'name': model_name+'_visual'})
        geometry_t = SubElement(visual_t, 'geometry')
        cylinder_t = SubElement(geometry_t, 'cylinder')
        radius_t = SubElement(cylinder_t, 'radius')
        radius_t.text = str(radius)
        length_t = SubElement(cylinder_t, 'length')
        length_t.text = str(length)

        self._add_material_tag(material, visual_t)

        self.spawn_sdf_model_client(model_name, tostring(sdf_t), ns=ns)

    def spawn_sphere(self, model_name, radius=0.5, mass=1.0,
                     xyz=None, rpy=None, material=None,
                     mu1=0.5, mu2=0.5, physics="ode", ns=None):
        if xyz is None:
            xyz = [0.0, 0.0, 0.0]
        if rpy is None:
            rpy = [0.0, 0.0, 0.0]
        if material is None:
            material = 'Gazebo/White'
        if ns is None:
            ns = self.gazebo_namespace
        if physics != 'ode':
            rospy.logwarn('The SDF generator currently generates physics'
                          'information for ODE only, other physics engines'
                          'have not been implemented yet.')

        ixx = mass*2/5*radius*radius
        iyy = mass*2/5*radius*radius
        izz = mass*2/5*radius*radius

        sdf_t = Element('sdf', {'version': '1.4'})
        model_t = SubElement(sdf_t, 'model', {'name': model_name})
        pose_t = SubElement(model_t, 'pose')
        pose_t.text = ' '.join([str(x) for x in xyz]) + ' ' + \
                      ' '.join([str(x) for x in rpy])
        link_t = SubElement(model_t, 'link', {'name': model_name})

        inertial_t = SubElement(link_t, 'inertial')
        mass_t = SubElement(inertial_t, 'mass')
        mass_t.text = str(mass)
        inertia_t = SubElement(inertial_t, 'inertia')
        SubElement(inertia_t, 'ixx').text = str(ixx)
        SubElement(inertia_t, 'ixy').text = '0'
        SubElement(inertia_t, 'ixz').text = '0'
        SubElement(inertia_t, 'iyy').text = str(iyy)
        SubElement(inertia_t, 'iyz').text = '0'
        SubElement(inertia_t, 'izz').text = str(izz)

        collision_t = SubElement(link_t, 'collision',
                                 {'name': model_name+'_collision'})
        geometry_t = SubElement(collision_t, 'geometry')
        sphere_t = SubElement(geometry_t, 'sphere')
        radius_t = SubElement(sphere_t, 'radius')
        radius_t.text = str(radius)
        surface_t = SubElement(collision_t, 'surface')
        friction_t = SubElement(surface_t, 'friction')
        ode_t = SubElement(friction_t, 'ode')
        SubElement(ode_t, 'mu').text = str(mu1)
        SubElement(ode_t, 'mu2').text = str(mu2)

        visual_t = SubElement(link_t, 'visual', {'name': model_name+'_visual'})
        geometry_t = SubElement(visual_t, 'geometry')
        sphere_t = SubElement(geometry_t, 'sphere')
        radius_t = SubElement(sphere_t, 'radius')
        radius_t.text = str(radius)

        self._add_material_tag(material, visual_t)

        self.spawn_sdf_model_client(model_name, tostring(sdf_t), ns=ns)

    @staticmethod
    def _add_material_tag(material, visual_t):
        material_t = SubElement(visual_t, 'material')
        script_t = SubElement(material_t, 'script')
        uri_t = SubElement(script_t, 'uri')
        uri_t.text = 'file://media/materials/scripts/gazebo.material'
        name_t = SubElement(script_t, 'name')
        name_t.text = material

    def spawn_sdf_model_client(self, model_name, model_xml,
                               robot_namespace=None, initial_pose=None,
                               reference_frame=None, ns=None):
        if robot_namespace is None:
            robot_namespace = rospy.get_namespace()
        if initial_pose is None:
            initial_pose = Pose()
            initial_pose.orientation.w = 1.0
        if reference_frame is None:
            reference_frame = ''
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns+'/spawn_sdf_model')
        try:
            spawn_sdf_model = rospy.ServiceProxy(ns+'/spawn_sdf_model',
                                                 SpawnModel)
            resp = spawn_sdf_model(model_name, model_xml, robot_namespace,
                                   initial_pose, reference_frame)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("couldn't spawn sdf " + model_name + ": " + str(e))

    def spawn_urdf_model_client(self, model_name, model_xml, robot_namespace,
                                initial_pose, reference_frame, ns=None):
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns+'/spawn_urdf_model')
        try:
            spawn_urdf_model = rospy.ServiceProxy(ns+'/spawn_urdf_model',
                                                  SpawnModel)
            resp = spawn_urdf_model(model_name, model_xml, robot_namespace,
                                    initial_pose, reference_frame)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("couldn't spawn urdf " + model_name + ": " + str(e))

    def set_model_configuration_client(self, model_name, model_param_name,
                                       joint_names, joint_positions, ns=None):
        if ns is None:
            ns = self.gazebo_namespace
        rospy.wait_for_service(ns+'/set_model_configuration')
        rospy.loginfo("temporary hack to **fix** the -J joint position option "
                      "(issue #93), sleeping for 1 second to avoid race "
                      "condition.")
        time.sleep(1)
        try:
            set_model_configuration = rospy.ServiceProxy(
                ns+'/set_model_configuration', SetModelConfiguration)
            rospy.loginfo("Calling service %s/set_model_configuration" % ns)
            resp = set_model_configuration(model_name, model_param_name,
                                           joint_names, joint_positions)
            rospy.loginfo("Set model configuration status: %s"
                          % resp.status_message)

            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("couldn't set model config " + model_name + ": "
                         + str(e))
