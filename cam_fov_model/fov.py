import bpy
from math import fabs, tan, radians, degrees, atan2
#import time

import random
import copy

import importlib as imp

class Cam:

    def __init__(self, focus, fov_distance, matrix_width, matrix_height):

        self.blend_cam_obj = None
        self.blend_cam_fov_obj = None

        # camera params [mm]
        self.focus = focus
        self.matrix_width = matrix_width
        self.matrix_height = matrix_height

        # fov params [m]
        self.fov_distance = fov_distance
        self.fov_width = (self.matrix_width*fabs(self.fov_distance))/self.focus
        self.fov_height = (self.matrix_height*fabs(self.fov_distance))/self.focus
        self.aov_horizontal = 2.0 * degrees(atan2(self.fov_width/2.0, self.fov_distance))
        self.aov_vertical = 2.0 * degrees(atan2(self.fov_height/2.0, self.fov_distance))


    def set_cam_fov_by_dist(self, dist):
        self.fov_distance = dist
        self.fov_width = (self.matrix_width*fabs(self.fov_distance))/self.focus
        self.fov_height = (self.matrix_height*fabs(self.fov_distance))/self.focus

        p0_fov = (self.fov_width/2,   self.fov_height/2, -self.fov_distance)
        p1_fov = (self.fov_width/2,  -self.fov_height/2, -self.fov_distance)
        p2_fov = (-self.fov_width/2, -self.fov_height/2, -self.fov_distance)
        p3_fov = (-self.fov_width/2,  self.fov_height/2, -self.fov_distance)

        self.blend_cam_fov_obj.data.vertices[0].co = p0_fov
        self.blend_cam_fov_obj.data.vertices[1].co = p1_fov
        self.blend_cam_fov_obj.data.vertices[2].co = p2_fov
        self.blend_cam_fov_obj.data.vertices[3].co = p3_fov

        return self.fov_width, self.fov_height


    def set_cam_focus_by_aov(self, theta):
        self.focus = self.matrix_width / 2.0 / tan(radians(theta/2.0))
        self.fov_width = (self.matrix_width*fabs(self.fov_distance))/self.focus
        self.fov_height = (self.matrix_height*fabs(self.fov_distance))/self.focus
        self.aov_horizontal = 2.0 * degrees(atan2(self.fov_width/2.0, self.fov_distance))
        self.aov_vertical = 2.0 * degrees(atan2(self.fov_height/2.0, self.fov_distance))

        p0_fov = (self.fov_width/2,   self.fov_height/2, -self.fov_distance)
        p1_fov = (self.fov_width/2,  -self.fov_height/2, -self.fov_distance)
        p2_fov = (-self.fov_width/2, -self.fov_height/2, -self.fov_distance)
        p3_fov = (-self.fov_width/2,  self.fov_height/2, -self.fov_distance)

        self.blend_cam_fov_obj.data.vertices[0].co = p0_fov
        self.blend_cam_fov_obj.data.vertices[1].co = p1_fov
        self.blend_cam_fov_obj.data.vertices[2].co = p2_fov
        self.blend_cam_fov_obj.data.vertices[3].co = p3_fov

        return self.fov_width, self.fov_height, self.focus


    def set_cam_focus(self, focus):
        self.focus = focus
        self.fov_width = (self.matrix_width*fabs(self.fov_distance))/self.focus
        self.fov_height = (self.matrix_height*fabs(self.fov_distance))/self.focus
        self.aov_horizontal = 2.0 * degrees(atan2(self.fov_width/2.0, self.fov_distance))
        self.aov_vertical = 2.0 * degrees(atan2(self.fov_height/2.0, self.fov_distance))

        p0_fov = (self.fov_width/2,   self.fov_height/2, -self.fov_distance)
        p1_fov = (self.fov_width/2,  -self.fov_height/2, -self.fov_distance)
        p2_fov = (-self.fov_width/2, -self.fov_height/2, -self.fov_distance)
        p3_fov = (-self.fov_width/2,  self.fov_height/2, -self.fov_distance)

        self.blend_cam_fov_obj.data.vertices[0].co = p0_fov
        self.blend_cam_fov_obj.data.vertices[1].co = p1_fov
        self.blend_cam_fov_obj.data.vertices[2].co = p2_fov
        self.blend_cam_fov_obj.data.vertices[3].co = p3_fov

        return self.fov_width, self.fov_height, self.focus


    def get_cam_fov(self):
        return self.fov_width, self.fov_height, self.aov_horizontal

    def get_cam_params(self):
        return {'fov_width': self.fov_width,
                'fov_height': self.fov_height,
                'aov_horizontal': self.aov_horizontal,
                'aov_vertical': self.aov_vertical,
                'fov_distance': self.fov_distance,
                'focus': self.focus,
                'matrix_width': self.matrix_width,
                'matrix_height': self.matrix_height}


class Cam_array:

    def __init__(self):
        self.cam_list = {}

    def add_cam_from_scene_by_name(self, name):
        blend_cam_obj = bpy.data.objects.get(name)
        blend_cam_fov_obj = bpy.data.objects.get(name + '_FoV')
        if (blend_cam_obj is None or blend_cam_fov_obj is None):
            return False


        self.cam_list[name].blend_cam_obj = blend_cam_obj
        self.cam_list[name].blend_cam_fov_obj = blend_cam_fov_obj
        self.cam_list[name].fov_width = fabs(self.cam_list[name].blend_cam_fov_obj.data.vertices[0].co.x) * 2.0
        self.cam_list[name].fov_height = fabs(self.cam_list[name].blend_cam_fov_obj.data.vertices[0].co.y) * 2.0
        self.cam_list[name].fov_distance = fabs(self.cam_list[name].blend_cam_fov_obj.data.vertices[0].co.z)
        self.cam_list[name].focus = self.cam_list[name].matrix_width * self.cam_list[name].fov_distance / self.cam_list[name].fov_width
        self.cam_list[name].aov_horizontal = 2.0 * degrees(atan2(self.cam_list[name].fov_width/2.0, self.cam_list[name].fov_distance))

        self.cam_list[name] = Cam()

        return True

    def get_random_cam_name(self):
        while True:
            rand_name = 'cam_' + str(random.randint(0, 900))
            if bpy.data.objects.get(rand_name) is None:
                return rand_name


    def create_cam(self, cam_num = 1):

        saved_3d_cursor_location = copy.deepcopy(bpy.context.scene.cursor.location)
        saved_3d_cursor_rotation_euler = copy.deepcopy(bpy.context.scene.cursor.rotation_euler)

        for i in range(self.cam_num):
            x = random.uniform(-3, 3)
            y = random.uniform(-0.45, 0.45)
            roll = random.uniform(1.3, 1.8)
            pitch = random.uniform(-1, 1)
            yaw = random.uniform(-1, 1)

            rand_name = ''
            while True:
                rand_name = 'Cam_' + str(random.randint(0, 9000))
                if bpy.data.objects.get(rand_name) is None:
                    break

            bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 1.8), rotation=(1.57, 0, -1.57))
            #bpy.context.object.show_axis = True
            bpy.context.object.name = rand_name
            #bpy.context.object.show_name = True

            self.cam_list[rand_name] = Cam()
            self.cam_list[rand_name].blend_cam_obj = bpy.context.object

            w, h, th = self.cam_list[rand_name].get_cam_fov()
            fov_distance = self.cam_list[rand_name].fov_distance
            p0_fov = (w/2,   h/2, -fov_distance)
            p1_fov = (w/2,  -h/2, -fov_distance)
            p2_fov = (-w/2, -h/2, -fov_distance)
            p3_fov = (-w/2,  h/2, -fov_distance)


            # In the mometn of the object creation it's location is setted by 3d cursor location
            bpy.context.scene.cursor.location = (0, 0, 0)

            bpy.ops.mesh.primitive_cone_add(vertices=4)
            bpy.context.object.name = rand_name + '_FoV'

            self.cam_list[rand_name].blend_cam_fov_obj = bpy.context.object

            bpy.context.object.active_material = bpy.data.materials.new(rand_name + '_FoV_Material')

            r = random.uniform(0.3, 0.95)
            g = random.uniform(0.3, 0.95)
            b = random.uniform(0.3, 0.95)
            bpy.context.object.active_material.diffuse_color = (r, g, b, 0.5)


            context_matrix_world = copy.deepcopy(bpy.context.object.matrix_world)
            vertex_local_co = copy.deepcopy(bpy.context.object.data.vertices[4].co)

            bpy.context.scene.cursor.location = bpy.context.object.matrix_world @ bpy.context.object.data.vertices[4].co
            bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
            bpy.context.object.parent = bpy.data.objects[rand_name]
            bpy.context.object.location = (0, 0, 0)
            bpy.context.object.rotation_euler = (0, 0, 0)
            bpy.context.object.data.vertices[0].co = p0_fov
            bpy.context.object.data.vertices[1].co = p1_fov
            bpy.context.object.data.vertices[2].co = p2_fov
            bpy.context.object.data.vertices[3].co = p3_fov

        bpy.context.scene.cursor.location = saved_3d_cursor_location
        bpy.context.scene.cursor.rotation_euler = saved_3d_cursor_rotation_euler


    def new_cam(self, cam_name = None, location=(0, 0, 1.8), rotation=(1.57, 0, -1.57), \
                focus = 6.0, fov_distance = 2, matrix_width = 5.784, matrix_height = 3.264):

        if cam_name is None:
            cam_name = self.get_random_cam_name()

        saved_3d_cursor_location = copy.deepcopy(bpy.context.scene.cursor.location)
        saved_3d_cursor_rotation_euler = copy.deepcopy(bpy.context.scene.cursor.rotation_euler)

        bpy.ops.mesh.primitive_cube_add(size=0.06, location=location, rotation=rotation)
        bpy.context.object.name = cam_name
        #bpy.context.object.show_name = True
        #bpy.context.object.show_axis = True
        bpy.context.object.active_material = bpy.data.materials.new(cam_name + '_material')
        bpy.context.object.active_material.diffuse_color = (0.1, 0.1, 0.1, 1.0)

        self.cam_list[cam_name] = Cam(focus, fov_distance, matrix_width, matrix_height)
        self.cam_list[cam_name].blend_cam_obj = bpy.context.object

        w, h, th = self.cam_list[cam_name].get_cam_fov()
        fov_distance = self.cam_list[cam_name].fov_distance
        p0_fov = (w/2,   h/2, -fov_distance)
        p1_fov = (w/2,  -h/2, -fov_distance)
        p2_fov = (-w/2, -h/2, -fov_distance)
        p3_fov = (-w/2,  h/2, -fov_distance)


        # In the mometn of the object creation it's location is setted by 3d cursor location
        bpy.context.scene.cursor.location = (0, 0, 0)

        bpy.ops.mesh.primitive_cone_add(vertices=4)
        bpy.context.object.name = cam_name + '_FoV'

        self.cam_list[cam_name].blend_cam_fov_obj = bpy.context.object

        bpy.context.object.active_material = bpy.data.materials.new(cam_name + '_FoV_Material')

        r = random.uniform(0.3, 0.95)
        g = random.uniform(0.3, 0.95)
        b = random.uniform(0.3, 0.95)
        bpy.context.object.active_material.diffuse_color = (r, g, b, 0.5)


        context_matrix_world = copy.deepcopy(bpy.context.object.matrix_world)
        vertex_local_co = copy.deepcopy(bpy.context.object.data.vertices[4].co)

        bpy.context.scene.cursor.location = bpy.context.object.matrix_world @ bpy.context.object.data.vertices[4].co
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        bpy.context.object.parent = bpy.data.objects[cam_name]
        bpy.context.object.location = (0, 0, 0)
        bpy.context.object.rotation_euler = (0, 0, 0)
        bpy.context.object.data.vertices[0].co = p0_fov
        bpy.context.object.data.vertices[1].co = p1_fov
        bpy.context.object.data.vertices[2].co = p2_fov
        bpy.context.object.data.vertices[3].co = p3_fov

        bpy.context.scene.cursor.location = saved_3d_cursor_location
        bpy.context.scene.cursor.rotation_euler = saved_3d_cursor_rotation_euler

        obj = self.cam_list[cam_name].blend_cam_obj
        rna_ui = obj.get('_RNA_UI')
        if rna_ui is None:
            obj['_RNA_UI'] = {}
            rna_ui = obj['_RNA_UI']

        obj["prop"] = (1.0, 0.0, 0.0, 1.0)
        obj["bool"] = 0

        rna_ui["bool"] = {"description":"Bool",
                          "default": True,
                          "min":0,
                          "max":1,
                          "soft_min":0,
                          "soft_max":1}

        rna_ui["prop"] = {"description": "Color Prop",
                          "default": (1.0, 0.0, 0.0, 0.0),
                          "min": 0.0,
                          "max": 1.0,
                          "soft_min":0.0,
                          "soft_max":1.0}


#bpy.ops.mesh.primitive_cone_add(vertices=4)
#bpy.context.active_object.name = 'my cone'

#bpy.context.object.location
#bpy.context.object.rotation_euler

#bpy.data.objects['Cam_'+str(i)].location

#bpy.data.objects['Cam_'+str(i)].select_set(True)
#bpy.context.object.matrix_world.translation
#bpy.context.object.data.vertices[4].co
#bpy.ops.object.origin_set(

#bpy.context.scene.objects["Cube"].select_set(False)
#bpy.context.scene.objects["Cube_ch"].select_set(True)

#for a in range(0, 0.2, 6.28):
#    x = math.cos(a)*2
#    y = math.sin(a)*2
#    bpy.ops.transform.translate(value = (x, y, 0))
#    time.sleep(5)


#context = bpy.context

#using ops

#bpy.ops.material.new()
#new_mat = bpy.data.materials[-1]  # the new material is the last one in the list
#new_mat.name = "NAME"

# using API

#new_mat = bpy.data.materials.new("NAME")
# because there is already a material with name NAME from the op this will have name NAME.00x
#print(new_mat.name)
# setting the diffuse color

#new_mat.diffuse.color = (0,0,0) # black


# assigning the new material to the active object
# assume you have a mesh object as context.object
# using ops

#bpy.ops.object.material_slot_add()

# the new slot will be the active one

#context.object.active_material = new_mat


# using API
# add the material to the mesh's materials

#mesh = context.object.data
#mesh.materials.append(new_mat)


#camera_list = ['camera_front_left_6mm', 'camera_front_right_6mm']
#camera_objects = {}

#for object in list(bpy.data.objects):
#    if object.name in camera_list:
#        camera_objects[object.name] = object

#print(camera_objects)
